library cmdr_teleop;

import 'dart:async';
import 'dart:io';
import 'dart:convert';
import 'dart:isolate';

import 'package:upcom-api/tab_backend.dart';

class CmdrTeleop extends Tab {
  static final List<String> names = ['upcom-teleop', 'UpDroid Teleop', 'Teleop'];

  static List videoOnlyNodes = [
    '/stereo/left',
    '/stereo/right',
    '/web_video_server'
  ];

  static List allNodes = [
    '/joy_cmdr',
//    '/teleop_twist_joy'
  ];

  Directory _uproot;
  Process _shell;
  List _nodes;

  CmdrTeleop(SendPort sp, List args) :
  super(CmdrTeleop.names, sp, args) {
    _uproot = new Directory(args[2]);

    _startRosNodes(false);
  }

  Future _startRosNodes(bool videoOnly) {
    Completer c = new Completer();
    Process.start('bash', ['-c', '. ${_uproot.path}/catkin_ws/devel/setup.bash && roslaunch upcom_teleop upcom_teleop.launch video_only:=${videoOnly.toString()}'], runInShell: true).then((process) {
      _shell = process;
      stdout.addStream(process.stdout);
      stderr.addStream(process.stderr);

      _nodes = videoOnly ? videoOnlyNodes : allNodes;

      // Keep checking every second to see if nodes are up.
//      new Timer.periodic(new Duration(seconds: 1), (t) {
//
//      });

      bool nodesUp = false;

      while (!nodesUp) {
        ProcessResult result = Process.runSync('bash', ['-c', '. ${_uproot.path}/catkin_ws/devel/setup.bash && rosnode list'], runInShell: true);
        String out = result.stdout;
        for (String node in _nodes) {
          if (!out.contains(node)) {
            print('missing: $node');
            break;
          }

          print('stdout: $out. nodes are up.');
//        t.cancel();
          nodesUp = true;
          mailbox.send(new Msg('NODES_UP'));
          c.complete();
        }
      }
    });

    return c.future;
  }

  void registerMailbox() {
    mailbox.registerEndPointHandler('/$refName/$id/controller/0', _handleGamepadInput);
  }

  void _handleGamepadInput(String endpoint, String s) {
    print(s);
    if (_shell != null) _shell.stdin.add(UTF8.encode(s));
  }

  void cleanup() {
    // TODO: this does not actually shut down the nodes, fix it!
    if (_shell != null) _shell.kill(ProcessSignal.SIGINT);

    // Workaround with rosnode kill.
    String nodeString = _nodes.toString().replaceAll(',', '');
    nodeString = nodeString.substring(1, nodeString.length - 1);
    Process.run('bash', ['-c', 'rosnode kill $nodeString'], runInShell: true);
  }
}