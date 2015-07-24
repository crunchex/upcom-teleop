library cmdr_teleop;

import 'dart:io';
import 'dart:convert';
import 'dart:isolate';

import 'package:upcom-api/tab_backend.dart';
import 'package:upcom-api/ros.dart';

class CmdrTeleop extends Tab {
  Process _shell;

  CmdrTeleop(int id, String workspacePath, SendPort sp, List args) :
  super(id, 'UpDroidTeleop', sp) {
    Workspace workspace = new Workspace(workspacePath);
//    Ros.runNode(workspace, 'ros_arduino_python joy_cmdr.launch');

    Process.start('bash', ['-c', '. ${workspace.path}/catkin_ws/devel/setup.bash && roslaunch ros_arduino_python joy_cmdr.launch'], runInShell: true).then((process) {
      _shell = process;
      stdout.addStream(process.stdout);
      stderr.addStream(process.stderr);
    });
  }

  void registerMailbox() {
    mailbox.registerEndPointHandler('/$guiName/$id/controller/0', _handleGamepadInput);
  }

  void _handleGamepadInput(String endpoint, String s) {
    _shell.stdin.add(UTF8.encode(s));
  }

  void cleanup() {
    _shell.kill();
  }
}