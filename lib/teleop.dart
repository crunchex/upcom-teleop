library cmdr_teleop;

import 'dart:io';
import 'dart:convert';
import 'dart:isolate';

import 'package:upcom-api/tab_backend.dart';
import 'package:upcom-api/ros.dart';

class CmdrTeleop extends Tab {
  static final List<String> names = ['upcom-teleop', 'UpDroid Teleop', 'Teleop'];
  Process _shell;

  CmdrTeleop(SendPort sp, List args) :
  super(CmdrTeleop.names, sp, args) {
    Workspace workspace = new Workspace(args[2]);
//    Ros.runNode(workspace, 'ros_arduino_python joy_cmdr.launch');

    Process.start('bash', ['-c', '. ${workspace.path}/catkin_ws/devel/setup.bash && roslaunch ros_arduino_python joy_cmdr.launch'], runInShell: true).then((process) {
      _shell = process;
      stdout.addStream(process.stdout);
      stderr.addStream(process.stderr);
    });
  }

  void registerMailbox() {
    mailbox.registerEndPointHandler('/$refName/$id/controller/0', _handleGamepadInput);
  }

  void _handleGamepadInput(String endpoint, String s) {
    _shell.stdin.add(UTF8.encode(s));
  }

  void cleanup() {
    _shell.kill();
  }
}