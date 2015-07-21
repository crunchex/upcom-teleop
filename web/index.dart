import 'dart:html';
import 'teleop.dart';

void main() {
  ScriptElement teleopJs = new ScriptElement()
  ..type = 'text/javascript'
  ..src = 'http://localhost:12060/upcom-teleop/teleop.js';
  document.body.children.add(teleopJs);

  teleopJs.onLoad.first.then((_) {
    new UpDroidTeleop(1, 1);
  });
}