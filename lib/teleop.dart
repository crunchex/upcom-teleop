library cmdr_teleop;

import 'dart:io';
import 'dart:convert';
import 'dart:isolate';

import 'package:upcom-api/ros/ros.dart';
import 'package:upcom-api/tab.dart';
import 'package:upcom-api/updroid_message.dart';

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

    mailbox.send(new Msg('TAB_READY'));
  }

  void registerMailbox() {
    mailbox.registerEndPointHandler('/$guiName/$id/controller/0', _handleGamepadInput);
  }

  void _handleGamepadInput(String s) {
    _shell.stdin.add(UTF8.encode(s));
  }

  void cleanup() {
    _shell.kill();
  }
}

void main(List args, SendPort interfacesSendPort) {
  Tab.main(interfacesSendPort, args, (id, path, port, args) => new CmdrTeleop(id, path, port, args));
}