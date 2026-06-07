/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "Home Robot: A ROS 2 Jazzy package for Service Robotics Course at University of León", "index.html", [
    [ "Available Commands", "index.html#autotoc_md6", null ],
    [ "Home Robot Navigation System - Command Reference", "md_docs_2commands.html", [
      [ "Requirements", "index.html#autotoc_md2", null ],
      [ "Quick start (Docker)", "index.html#autotoc_md3", null ],
      [ "Hardware (Arduino)", "index.html#autotoc_md4", null ],
      [ "Planning Systems", "index.html#autotoc_md5", null ],
      [ "🐳 Running with Docker (Recommended)", "md_docs_2commands.html#autotoc_md9", [
        [ "1. Start/Stop the System", "md_docs_2commands.html#autotoc_md10", null ],
        [ "2. Access the VNC Desktop", "md_docs_2commands.html#autotoc_md11", null ]
      ] ],
      [ "🚀 Navigation &amp; Simulation (Inside Container)", "md_docs_2commands.html#autotoc_md13", [
        [ "2.1 Start Full Navigation Stack", "md_docs_2commands.html#autotoc_md14", null ],
        [ "2.2 Overhead Camera (Orthographic View)", "md_docs_2commands.html#autotoc_md15", null ]
      ] ],
      [ "🎤 Voice Control &amp; Patrols", "md_docs_2commands.html#autotoc_md17", [
        [ "3.1 Launch Voice Controller", "md_docs_2commands.html#autotoc_md18", null ],
        [ "3.2 Automated Waypoint Patrol (Legacy)", "md_docs_2commands.html#autotoc_md19", null ],
        [ "3.3 Advanced FSM Patrol (YASMIN)", "md_docs_2commands.html#autotoc_md20", null ]
      ] ],
      [ "🛠️ Hardware &amp; Tools", "md_docs_2commands.html#autotoc_md21", [
        [ "4.1 LiDAR Grid (Arduino Interface)", "md_docs_2commands.html#autotoc_md22", null ],
        [ "4.2 Save Map", "md_docs_2commands.html#autotoc_md23", null ],
        [ "4.3 Manual Control (Teleop)", "md_docs_2commands.html#autotoc_md24", null ]
      ] ]
    ] ],
    [ "FSM Patrol Architecture (YASMIN)", "md_docs_2fsm__info.html", [
      [ "🧭 General Flow", "md_docs_2fsm__info.html#autotoc_md27", [
        [ "1. <span class=\"tt\">SELECT_WAYPOINT</span>", "md_docs_2fsm__info.html#autotoc_md28", null ],
        [ "2. <span class=\"tt\">NAVIGATE</span>", "md_docs_2fsm__info.html#autotoc_md29", null ],
        [ "3. <span class=\"tt\">WAYPOINT_REACHED</span>", "md_docs_2fsm__info.html#autotoc_md30", null ],
        [ "4. <span class=\"tt\">HANDLE_ERROR</span>", "md_docs_2fsm__info.html#autotoc_md31", null ]
      ] ],
      [ "🚧 Active Anti-Stuck System", "md_docs_2fsm__info.html#autotoc_md33", [
        [ "Problem", "md_docs_2fsm__info.html#autotoc_md34", null ],
        [ "The Solution: <span class=\"tt\">CHECK_STUCK</span> &amp; <span class=\"tt\">STUCK_RECOVERY</span>", "md_docs_2fsm__info.html#autotoc_md35", null ]
      ] ]
    ] ],
    [ "Hardware Components for Home Robot Arduino-based Hardware Interface", "md_docs_2hardware-components.html", null ],
    [ "Patrol Planners", "md_docs_2planners.html", [
      [ "Visualizers", "md_docs_2planners.html#autotoc_md38", null ],
      [ "1. YASMIN FSM", "md_docs_2planners.html#autotoc_md39", null ],
      [ "2. BehaviorTree.CPP", "md_docs_2planners.html#autotoc_md40", null ],
      [ "3. PDDL (POPF)", "md_docs_2planners.html#autotoc_md41", null ],
      [ "Comparison", "md_docs_2planners.html#autotoc_md42", null ],
      [ "Typical Demo", "md_docs_2planners.html#autotoc_md43", null ]
    ] ],
    [ "Packages", "namespaces.html", [
      [ "Package List", "namespaces.html", "namespaces_dup" ],
      [ "Package Members", "namespacemembers.html", [
        [ "All", "namespacemembers.html", null ],
        [ "Functions", "namespacemembers_func.html", null ],
        [ "Variables", "namespacemembers_vars.html", null ]
      ] ]
    ] ],
    [ "Classes", "annotated.html", [
      [ "Class List", "annotated.html", "annotated_dup" ],
      [ "Class Index", "classes.html", null ],
      [ "Class Hierarchy", "hierarchy.html", "hierarchy" ],
      [ "Class Members", "functions.html", [
        [ "All", "functions.html", null ],
        [ "Functions", "functions_func.html", null ],
        [ "Variables", "functions_vars.html", null ],
        [ "Typedefs", "functions_type.html", null ]
      ] ]
    ] ],
    [ "Files", "files.html", [
      [ "File List", "files.html", "files_dup" ],
      [ "File Members", "globals.html", [
        [ "All", "globals.html", null ],
        [ "Functions", "globals_func.html", null ],
        [ "Variables", "globals_vars.html", null ],
        [ "Macros", "globals_defs.html", null ]
      ] ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"Dockerfile.html",
"classhome__robot__bt_1_1bt__patrol_1_1NavigateToWaypoint.html",
"namespacehome__robot_1_1overhead__cam__service.html"
];

const SYNCONMSG = 'click to disable panel synchronization';
const SYNCOFFMSG = 'click to enable panel synchronization';
const LISTOFALLMEMBERS = 'List of all members';