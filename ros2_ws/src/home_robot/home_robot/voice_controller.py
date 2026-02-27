#!/usr/bin/env python3
"""
Voice Controller Node - Spanish voice command interface for robot navigation.

This node provides complete voice control using Vosk STT (speech-to-text) and
Piper TTS (text-to-speech). It supports navigation to predefined locations,
coordinate-based navigation, patrol commands, map saving, and stop commands.

Voice Commands (in Spanish):
    - "ve a [location]" - Navigate to predefined location
    - "ve a [x] [y]" - Navigate to coordinates
    - "patrulla" - Start sequential waypoint patrol
    - "patrulla aleatoria" - Start random waypoint patrol
    - "para" / "stop" - Stop all navigation and patrol
    - "guardar mapa" - Save current SLAM map
    - "opciones" - List available commands

Predefined Locations:
    sofá, cocina, almacén, principal, baño, secundaria, inicio

Requirements:
    - Python virtual environment at PROJECT_ROOT/.venv
    - Vosk model: vosk-model-small-es-0.42
    - Piper TTS: es_ES-carlfm-x_low.onnx
    - sounddevice, vosk packages
    - Microphone and speakers

Usage:
    # Launch with dedicated script (activates venv)
    ./scripts/launch_voice.sh
    
    # Or directly (ensure venv is activated)
    ros2 run home_robot voice_controller

Architecture:
    - STT thread: Continuous audio capture and recognition
    - TTS thread: Non-blocking speech synthesis
    - Navigation thread: Non-blocking BasicNavigator operations
    - Main thread: ROS2 executor for patrol event subscriptions
"""

import os
import sys
import subprocess
import json
import queue
import threading
import time
import signal

# --- DYNAMIC PATH DETECTION ---
# Try to get project root from environment, fallback to a relative path from this script
PROJECT_ROOT = os.environ.get("PROJECT_ROOT")
if not PROJECT_ROOT:
    # Fallback to logical parent directory if not in environment
    # Assumes script is in <root>/ros2_ws/src/home_robot/home_robot/
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, "../../../.."))

# We don't hardcode venv injection here anymore. 
# Users should activate venv before running or we rely on system/container packages.
# ------------------------------

import rclpy
from rclpy.node import Node
import sounddevice as sd
from vosk import Model, KaldiRecognizer
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String

from rclpy.executors import MultiThreadedExecutor


class VoiceController(Node):
    """
    ROS2 node for Spanish voice-controlled robot navigation.
    
    Integrates Vosk STT, Piper TTS, and Nav2 navigation stack to provide
    hands-free control of a mobile robot. Handles concurrent audio processing,
    navigation management, and patrol coordination.
    
    Attributes:
        piper_path (str): Path to Piper TTS executable
        vosk_model_path (str): Path to Vosk STT model
        locations (dict): Named navigation waypoints
        numero_map (dict): Spanish number words to numeric conversion
        is_speaking (bool): TTS active flag
        is_navigating (bool): Navigation active flag
        patrol_process (subprocess.Popen): Active patrol subprocess
    """
    
    def __init__(self):
        super().__init__('voice_controller_node')
        
        # Model and Binary Paths
        self.piper_path = os.path.join(PROJECT_ROOT, "voice_models/piper/piper")
        self.piper_model = os.path.join(PROJECT_ROOT, "voice_models/piper/voices/es_ES-carlfm-x_low.onnx")
        self.piper_config = os.path.join(PROJECT_ROOT, "voice_models/piper/voices/es_ES-carlfm-x_low.onnx.json")
        self.vosk_model_path = os.path.join(PROJECT_ROOT, "voice_models/vosk/vosk-model-small-es-0.42")

        # State Management
        self.is_speaking = False
        self.patrol_process = None  # Active patrol subprocess reference
        self.nav_thread = None  # Active navigation thread
        self.is_navigating = False  # Navigation active flag

        # 1. Configure STT (Vosk)
        self.get_logger().info("Cargando modelo Vosk...")
        self.vosk_model = Model(self.vosk_model_path)
        self.rec = KaldiRecognizer(self.vosk_model, 16000)
        self.audio_queue = queue.Queue()

        # 2. Configure Navigation (BasicNavigator without passing node to avoid spin conflicts)
        self.nav = BasicNavigator()
        
        # Publisher to immediately stop robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher to clear visual path (without affecting costmaps)
        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        
        # Subscriber for patrol events (waypoint arrivals)
        self.patrol_event_sub = self.create_subscription(
            String,
            'patrol_events',
            self.patrol_event_callback,
            10
        )
        
        # Publisher for emergency stop request (handles execute_patrol termination)
        self.stop_pub = self.create_publisher(String, '/stop_patrol', 10)
        
        # 3. Navigation Destinations
        self.locations = {
            "sofá": [3.5, -2.5],
            "sofa": [3.5, -2.5],
            "sillón": [3.5, -2.5],
            "cocina": [8.0, -3.0],
            "almacén": [12.0, -1.0],
            "almacen": [12.0, -1.0],
            "principal": [10.0, 6.0],
            "baño": [5.0, 6.0],
            "banyo": [5.0, 6.0],
            "secundaria": [1.0, 5.0],
            "casa": [0.0, 0.0],
            "inicio": [0.0, 0.0],
            "origen": [0.0, 0.0]
        }
        
        # Spanish number word to numeric conversion dictionary
        self.numero_map = {
            "cero": 0, "uno": 1, "dos": 2, "tres": 3, "cuatro": 4,
            "cinco": 5, "seis": 6, "siete": 7, "ocho": 8, "nueve": 9,
            "diez": 10, "once": 11, "doce": 12, "trece": 13, "catorce": 14,
            "quince": 15, "dieciséis": 16, "dieciseis": 16, "diecisiete": 17,
            "dieciocho": 18, "diecinueve": 19, "veinte": 20,
            "menos": "-"
        }

        self.say("¿A dónde vamos?")
        
        # Start continuous voice listening thread
        threading.Thread(target=self.voice_loop, daemon=True).start()

    def say(self, text: str) -> None:
        """
        Synthesize and play Spanish text using Piper TTS.
        
        Non-blocking method that runs TTS in a separate thread. Sets is_speaking
        flag to prevent audio input interference during speech output.
        
        Args:
            text: Spanish text to synthesize and speak
        """
        def _speak():
            self.is_speaking = True
            self.get_logger().info(f"Diciendo: {text}")
            try:
                # Verifica binario y permiso antes de hablar
                if not os.path.isfile(self.piper_path):
                    self.get_logger().error(f"Piper no encontrado en {self.piper_path}")
                    return
                if not os.access(self.piper_path, os.X_OK):
                    self.get_logger().error("Piper no es ejecutable; ejecuta chmod +x sobre el binario")
                    return

                # Generate audio with Piper and play with aplay
                env = os.environ.copy()
                env.setdefault("OMP_NUM_THREADS", "1")
                env.setdefault("OMP_WAIT_POLICY", "PASSIVE")

                run_tts = subprocess.run(
                    [self.piper_path, "--model", self.piper_model, "--config", self.piper_config, "--output_file", "/tmp/tts.wav"],
                    input=text.encode('utf-8'),
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    env=env,
                    check=False
                )

                if run_tts.returncode != 0:
                    err_text = run_tts.stderr.decode('utf-8', errors='ignore') if run_tts.stderr else ''
                    self.get_logger().error(f"Piper falló (código={run_tts.returncode}): {err_text[:200]}")
                    return

                play = subprocess.run(["aplay", "/tmp/tts.wav"], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, env=env, check=False)
                if play.returncode != 0:
                    err_text = play.stderr.decode('utf-8', errors='ignore') if play.stderr else ''
                    self.get_logger().error(f"aplay falló (código={play.returncode}): {err_text[:200]}")
                    return
            except Exception as e:
                self.get_logger().error(f"Error en TTS: {e}")
            finally:
                time.sleep(0.5)
                self.is_speaking = False

        threading.Thread(target=_speak).start()
    
    def patrol_event_callback(self, msg: String) -> None:
        """
        Handle patrol waypoint arrival events.
        
        Called when execute_patrol publishes waypoint arrival notifications.
        Announces arrivals via TTS.
        
        Args:
            msg: String message with format "llegada:<waypoint_name>"
        """
        try:
            self.get_logger().info(f"📬 Evento de patrulla recibido: {msg.data}")
            if msg.data.startswith("llegada:"):
                waypoint_name = msg.data.split(":", 1)[1]
                self.get_logger().info(f"🎯 Anunciando llegada a: {waypoint_name}")
                self.say(f"He llegado a {waypoint_name}")
        except Exception as e:
            self.get_logger().error(f"Error en patrol_event_callback: {e}", exc_info=True)

    def audio_callback(self, indata, frames, time, status) -> None:
        """
        Audio stream callback for continuous microphone input.
        
        Called by sounddevice for each audio block. Queues audio data for
        processing unless TTS is currently speaking.
        
        Args:indata: Audio samples from microphone
            frames: Number of frames
            time: Timing information
            status: Stream status flags
        """
        if not self.is_speaking:
            self.audio_queue.put(bytes(indata))
        else:
            _ = self.rec.PartialResult()

    def voice_loop(self) -> None:
        """
        Continuous voice recognition loop.
        
        Runs in separate thread, processing queued audio data through Vosk
        recognizer and dispatching recognized commands.
        """
        try:
            with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                                  channels=1, callback=self.audio_callback):
                self.get_logger().info("🎤 Escuchando con Vosk (Offline)...")
                while rclpy.ok():
                    data = self.audio_queue.get()
                    if self.rec.AcceptWaveform(data):
                        result = json.loads(self.rec.Result())
                        text = result.get("text", "").lower()
                        if text:
                            self.get_logger().info(f"Entendido: '{text}'")
                            self.process_command(text)
        except Exception as e:
            self.get_logger().error(f"Error en bucle de voz: {e}")

        try:
            self.get_logger().info("Limpiando path visual...")
            empty_path = Path()
            empty_path.header.frame_id = 'map'
            empty_path.header.stamp = self.get_clock().now().to_msg()
            self.plan_pub.publish(empty_path)
        except Exception as e:
            self.get_logger().warn(f"No se pudo limpiar path visual: {e}")
    
    def process_command(self, text: str) -> None:
        """
        Process recognized voice command and execute corresponding action.
        
        Handles stop commands, location navigation, coordinate navigation,
        patrol commands, map saving, and options listing.
        
        Args:
            text: Recognized text from voice input (lowercase Spanish)
        """
        # Detect stop command (with various recognition variations)
        if any(w in text for w in ["para", "pará", "parar", "stop", "detente", "detén", "alto"]):
            self.get_logger().info("🛑 Comando de parada detectado")
            
            # STEP 0: Send ROS-native stop signal first (most robust)
            stop_msg = String()
            stop_msg.data = "STOP"
            self.stop_pub.publish(stop_msg)
            self.get_logger().info("Señal de parada enviada por tópico /stop_patrol")

            # STEP 1: Send SIGTERM to execute_patrol...
            try:
                # First SIGTERM to allow cleanup opportunity
                result = subprocess.run(
                    ["pkill", "-TERM", "-f", "execute_patrol"],
                    capture_output=True,
                    timeout=1
                )
                self.get_logger().info(f"SIGTERM enviado (código: {result.returncode})")
                
                # Wait 0.5s for signal processing
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().warn(f"Error enviando SIGTERM: {e}")
            
            # STEP 2: If registered process still alive, kill with SIGTERM
            if self.patrol_process:
                try:
                    if self.patrol_process.poll() is None:  # If still alive
                        self.get_logger().info(f"Terminando PID registrado con SIGTERM: {self.patrol_process.pid}")
                        # Send SIGTERM to session group
                        os.killpg(os.getpgid(self.patrol_process.pid), signal.SIGTERM)
                        time.sleep(0.3)
                except:
                    pass
            
            # STEP 3: If STILL alive, use SIGKILL
            try:
                result = subprocess.run(
                    ["pkill", "-9", "-f", "execute_patrol"],
                    capture_output=True,
                    timeout=1
                )
                if result.returncode == 0:
                    self.get_logger().info("SIGKILL enviado como respaldo")
            except:
                pass
            
            # STEP 4: Clean process reference
            if self.patrol_process:
                try:
                    self.patrol_process.wait(timeout=0.2)
                except:
                    pass
                self.patrol_process = None
            
            # STEP 5: Mark local navigation as inactive (the thread will handle cancellation)
            self.is_navigating = False
            
            # STEP 6: Clear visual path
            self.cancel_all_navigation()
            
            # STEP 7: Publish zero velocity MULTIPLE TIMES (immediate physical stop)
            self.get_logger().info("Publicando cmd_vel cero (frenado inmediato)...")
            stop_msg = Twist()
            for _ in range(20):
                self.cmd_vel_pub.publish(stop_msg)
                time.sleep(0.01)
            
            self.get_logger().info("✓ Robot detenido completamente")
            self.say("Entendido, me detengo.")
            return

        if "opciones" in text or "opción" in text:
            # List main destinations (without synonyms)
            destinos = ["sofá", "cocina", "almacén", "principal", "baño", "secundaria", "inicio"]
            mensaje = "Puedo ir a los siguientes lugares: " + ", ".join(destinos) + ". También puedo ir a unas coordenadas específicas, hacer una patrulla, una patrulla aleatoria, o guardar el mapa."
            self.say(mensaje)
            return

        # Detect map save command
        if "guardar" in text and "mapa" in text:
            self.say("Guardando el mapa.")
            self.save_map()
            return

        # Detect patrol command
        if "patrulla" in text:
            # Cancel active navigation before starting patrol
            if self.is_navigating:
                self.get_logger().info("Cancelando navegación para iniciar patrulla...")
                self.nav.cancelTask()
                self.is_navigating = False
                time.sleep(0.3)
            
            if "aleatorio" in text or "aleatoria" in text or "random" in text:
                self.say("Iniciando patrulla aleatoria.")
                self.launch_patrol(random=True)
            else:
                self.say("Iniciando patrulla secuencial.")
                self.launch_patrol(random=False)
            return

        # Detect coordinate commands (e.g., "ve a uno uno", "ir a dos tres")
        coords = self.extract_coordinates(text)
        if coords:
            x, y = coords
            self.say(f"Entendido. Voy a las coordenadas {x}, {y}")
            self.go_to([x, y], f"coordenadas {x}, {y}")
            return

        for key, coords in self.locations.items():
            if key in text:
                self.say(f"Entendido. Voy a {key}")
                self.go_to(coords, key)
                return

    def extract_coordinates(self, text: str):
        """
        Extract numeric coordinates from Spanish voice command.
        
        Converts Spanish number words to floats for navigation.
        Examples:
            "ve a uno dos" -> (1.0, 2.0)
            "ir a menos tres cinco" -> (-3.0, 5.0)
        
        Args:
            text: Recognized Spanish voice command
            
        Returns:
            tuple: (x, y) coordinates as floats, or None if not found
        """
        import re
        
        # Look for patterns like "ve a", "ir a", "coordenadas"
        if not any(pattern in text for pattern in ["ve a", "ir a", "coordenadas"]):
            return None
        
        # Extract words after command
        words = text.split()
        numbers = []
        i = 0
        sign = 1
        
        while i < len(words):
            word = words[i]
            
            # Detect negative sign
            if word == "menos":
                sign = -1
                i += 1
                continue
            
            # Convert word to number
            if word in self.numero_map:
                num_value = self.numero_map[word]
                if isinstance(num_value, int):
                    numbers.append(float(sign * num_value))
                    sign = 1
            # Detect numbers written as digits
            elif word.isdigit() or (word.replace('-', '').replace('.', '').isdigit()):
                try:
                    numbers.append(float(word))
                    sign = 1
                except:
                    pass
            
            i += 1
        
        # Need at least 2 numbers (x, y)
        if len(numbers) >= 2:
            return (numbers[0], numbers[1])
        
        return None

    def go_to(self, coords: list, name: str) -> None:
        """
        Navigate to specified coordinates using Nav2.
        
        Cancels any active navigation/patrol before starting new goal.
        Runs navigation in separate thread to avoid blocking voice recognition.
        
        Args:
            coords: [x, y] coordinates in map frame
            name: Descriptive name for TTS announcements
        """
        # Cancel previous navigation if exists
        if self.is_navigating:
            self.get_logger().info("Cancelando navegación anterior...")
            self.nav.cancelTask()
            time.sleep(0.5)  # Wait longer for cancellation to process completely
        
        # Cancel active patrol process if exists
        if self.patrol_process and self.patrol_process.poll() is None:
            self.get_logger().info(f"Terminando proceso de patrulla (PID: {self.patrol_process.pid})")
            self.patrol_process.terminate()
            time.sleep(0.3)
            if self.patrol_process.poll() is None:
                self.patrol_process.kill()
            self.patrol_process = None
        
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(coords[0])
        goal.pose.position.y = float(coords[1])
        goal.pose.orientation.w = 1.0

        def _nav_task():
            self.is_navigating = True
            try:
                # Send goal
                self.nav.goToPose(goal)
                
                # Verify future exists
                if not hasattr(self.nav, 'result_future') or self.nav.result_future is None:
                    self.get_logger().error(f"No se pudo iniciar navegación a {name}")
                    self.say("Error al iniciar la navegación.")
                    return
                
                # Wait for navigation to complete
                while not self.nav.isTaskComplete() and self.is_navigating:
                    time.sleep(0.1)
                
                # If exited via manual cancellation, perform actual task cancellation here (thread-safe)
                if not self.is_navigating:
                    self.get_logger().info("Cancelando tarea en Nav2 de forma segura...")
                    try:
                        self.nav.cancelTask()
                    except:
                        pass
                    return
                
                # Get result
                result = self.nav.getResult()
                
                # Check if we are close enough even if result isn't perfect
                # (Simulation sometimes returns FAILED or UNKNOWN if tolerances are tight)
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f"✓ Navegación a {name} completada con éxito")
                    self.say(f"Ya he llegado a {name}")
                else:
                    self.get_logger().warn(f"Navegación a {name} terminó con resultado: {result}")
                    # If we finished the loop, we are likely at the goal even if result isn't 'Succeeded'
                    self.say(f"He llegado a {name}")
            except KeyboardInterrupt:
                self.get_logger().info("Navegación interrumpida por el usuario")
                self.nav.cancelTask()
            except Exception as e:
                self.get_logger().error(f"Error en navegación: {e}", exc_info=True)
                try:
                    self.say("Ocurrió un error durante la navegación.")
                except:
                    pass  # If TTS fails, at least don't block
            finally:
                self.is_navigating = False

        self.nav_thread = threading.Thread(target=_nav_task, daemon=True)
        self.nav_thread.start()

    def launch_patrol(self, random: bool = False) -> None:
        """
        Launch execute_patrol node in sequential or random mode.
        """
        # Cancel any previous patrol first
        if self.patrol_process and self.patrol_process.poll() is None:
            self.get_logger().info("Deteniendo patrulla previa...")
            os.killpg(os.getpgid(self.patrol_process.pid), signal.SIGTERM)
            time.sleep(0.2)

        try:
            cmd = ["ros2", "run", "home_robot", "execute_patrol"]
            if random:
                cmd.append("--random")
            
            self.get_logger().info(f"🚀 Lanzando patrulla: {' '.join(cmd)}")
            
            # Use os.setsid to create a process group that we can kill later
            self.patrol_process = subprocess.Popen(
                cmd,
                env=os.environ.copy(),
                preexec_fn=os.setsid,
                stdout=None,
                stderr=None
            )
            self.get_logger().info(f"✓ Patrulla iniciada (PID: {self.patrol_process.pid})")
            
        except Exception as e:
            self.get_logger().error(f"❌ Error al lanzar patrulla: {e}")
            self.say("Error al iniciar la patrulla")

    def save_map(self) -> None:
        """
        Save current SLAM map using SLAM Toolbox service.
        
        Generates timestamped map file in ros2_ws/src/home_robot/maps/.
        Runs asynchronously in separate thread.
        """
        def _save_task():
            try:
                # Generate map name with timestamp
                from datetime import datetime
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                map_name = os.path.join(PROJECT_ROOT, "ros2_ws/src/home_robot/maps", f"mapa_{timestamp}")
                
                # Build command
                cmd = [
                    "ros2", "service", "call",
                    "/slam_toolbox/save_map",
                    "slam_toolbox/srv/SaveMap",
                    f"{{name: {{data: '{map_name}'}}}}"
                ]
                
                self.get_logger().info(f"Guardando mapa: {map_name}")
                
                # Execute command
                result = subprocess.run(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    env=os.environ.copy(),
                    timeout=10
                )
                
                if result.returncode == 0:
                    self.get_logger().info(f"Mapa guardado exitosamente en {map_name}")
                    self.say("Mapa guardado correctamente.")
                else:
                    error_msg = result.stderr.decode('utf-8', errors='ignore')
                    self.get_logger().error(f"Error al guardar mapa: {error_msg}")
                    self.say("Error al guardar el mapa.")
                
            except subprocess.TimeoutExpired:
                self.get_logger().error("Timeout al guardar el mapa")
                self.say("El guardado del mapa tardó demasiado.")
            except Exception as e:
                self.get_logger().error(f"Error al guardar mapa: {e}")
                self.say("Error al guardar el mapa.")
        
        threading.Thread(target=_save_task, daemon=True).start()


def main():
    """
    Main entry point for voice controller node.
    
    Initializes ROS2, creates VoiceController node, and runs with
    MultiThreadedExecutor to handle concurrent callbacks.
    """
    rclpy.init()
    node = VoiceController()
    # Use MultiThreadedExecutor to allow node to handle voice thread and
    # navigation callbacks simultaneously without blocking
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("\n🛑 Ctrl-C detectado, cerrando voice_controller...")
        # Stop patrol processes if active
        if node.patrol_process and node.patrol_process.poll() is None:
            node.get_logger().info("Terminando patrulla activa...")
            try:
                import os, signal
                os.killpg(os.getpgid(node.patrol_process.pid), signal.SIGTERM)
            except:
                pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


