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
//      stdout.addStream(process.stdout);
//      stderr.addStream(process.stderr);

      _nodes = videoOnly ? videoOnlyNodes : allNodes;

      bool nodesUp = false;

      while (!nodesUp) {
        ProcessResult result = Process.runSync('bash', ['-c', '. ${_uproot.path}/catkin_ws/devel/setup.bash && rosnode list'], runInShell: true);
        String out = result.stdout;
        for (String node in _nodes) {
          if (!out.contains(node)) break;

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
    if (s == '') return;

    String joyInput = _processJoy(s);
    if (_shell != null) _shell.stdin.add(UTF8.encode(joyInput));
  }

  /// Swaps the fourth comma for a semicolon as required by the joy_cmdr node.
  String _processJoy(String s) {
    List l = _remapJoy(JSON.decode(s));

    List axes = l.sublist(0, 8);
    List buttons = l.sublist(8, l.length);

    String axesString = JSON.encode(axes);
    axesString = axesString.substring(0, axesString.length - 1);

    String buttonsString = JSON.encode(buttons);
    buttonsString = buttonsString.substring(1, buttonsString.length);

    return axesString + ';' + buttonsString;
  }

  List _remapJoy(Map<String, List> raw) {
    //  Table of index number of /joy.axis:
    //
    //  0 - Left/Right Axis stick left
    //  1 - Up/Down Axis stick left
    //  2 - LT
    //  3 - Left/Right Axis stick right
    //  4 - Up/Down Axis stick right
    //  5 - RT
    //  6 - cross key left/right
    //  7 - cross key up/down
    //
    //  Table of index number of /joy.buttons:
    //
    //  0 - A
    //  1 - B
    //  2 - X
    //  3 - Y
    //  4 - LB
    //  5 - RB
    //  6 - back
    //  7 - start
    //  8 - power
    //  9 - Button stick left
    //  10 - Button stick right

    List l = [];

    // Axes.
    l.addAll(raw['axes'].sublist(0, 2));
    l.add(raw['buttons'][6] == 1 ? 1 : -1);
    l.addAll(raw['axes'].sublist(2, raw['axes'].length));
    l.add(raw['buttons'][7] == 1 ? 1 : -1);

    // Convert D-L/R to an axis.
    if (raw['buttons'][14] == 1) {
      l.add(-1);
    } else if (raw['buttons'][15] == 1) {
      l.add(1);
    } else {
      l.add(0);
    }

    // Convert D-U/D to an axis.
    if (raw['buttons'][13] == 1) {
      l.add(-1);
    } else if (raw['buttons'][12] == 1) {
      l.add(1);
    } else {
      l.add(0);
    }

    // Buttons.
    l.addAll(raw['buttons'].sublist(0, 6));
    l.addAll(raw['buttons'].sublist(8, 10));
    l.add(raw['buttons'].last);
    l.addAll(raw['buttons'].sublist(10, 12));

    return l;
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