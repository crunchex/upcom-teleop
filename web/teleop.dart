library updroid_teleop;

import 'dart:html';
import 'dart:async';
import 'dart:js' as js;

import 'package:upcom-api/web/tab/tab_controller.dart';

class UpDroidTeleop extends TabController {
  static final List<String> names = ['upcom-teleop', 'UpDroid Teleop', 'Teleop'];

  static List getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'toggle', 'title': 'Close Tab'}]},
      {'title': 'Controllers', 'items': []}
    ];
    return menu;
  }

  DivElement containerDiv;

  WebSocket _ws;

  UpDroidTeleop() :
  super(UpDroidTeleop.names, getMenuConfig(), 'tabs/upcom-teleop/teleop.css') {

  }

  void setUpController() {
    // Dummy div so that teleop fits with the onFocus API from tab controller.
    containerDiv = new DivElement()
    ..style.width = '100%'
    ..style.height = '100%'
    ..style.backgroundColor = '#107C10'
    ..style.outline = 'none'
    ..tabIndex = -1;
    view.content.children.add(containerDiv);

    view.content.contentEdge.height = new Dimension.percent(100);

    // TODO: compress this svg (use that OS X tool).
    ImageElement image = new ImageElement(src:'tabs/$refName/xbox.svg')
      ..id = '$refName-$id-icon'
      ..classes.add('$refName-icon');
    containerDiv.children.add(image);

    for (int i = 0; i < 4; i++) {
      SpanElement span = new SpanElement()
        ..id = '$refName-$id-axis-span-$i'
        ..classes.add('$refName-axis-span')
        ..style.transform = 'translate(-50%, -${i * 20 + 50}px)';
      containerDiv.children.add(span);

      ParagraphElement axisLabel = new ParagraphElement()
        ..id = '$refName-$id-axis-label-$i'
        ..classes.add('$refName-axis-label')
        ..text = 'Axis $i: ';
      span.children.add(axisLabel);

      ParagraphElement axisData = new ParagraphElement()
        ..id = '$refName-$id-axis-data-$i'
        ..classes.add('$refName-axis-data')
        ..text = 'disconnected';
      span.children.add(axisData);
    }

    for (int i = 0; i < 17; i++) {
      SpanElement span = new SpanElement()
        ..id = '$refName-$id-button-span-$i'
        ..classes.add('$refName-button-span')
        ..style.transform = 'translate(-50%, ${i * 20 - 30}px)';
      containerDiv.children.add(span);

      ParagraphElement buttonLabel = new ParagraphElement()
        ..id = '$refName-$id-button-label-$i'
        ..classes.add('$refName-button-label')
        ..text = 'Button $i: ';
      span.children.add(buttonLabel);

      ParagraphElement buttonData = new ParagraphElement()
        ..id = '$refName-$id-button-data-$i'
        ..classes.add('$refName-button-data')
        ..text = '0';
      span.children.add(buttonData);
    }

    new js.JsObject(js.context['startScanning'], [id]);

    String url = window.location.host;
    url = url.split(':')[0];
    // window.location.host returns whatever is in the URL bar (including port).
    // Since the port here needs to be dynamic, the default needs to be replaced.
    _initWebSocket('ws://' + url + ':12060/$refName/$id/controller/0');

    //_setGamepads();
  }

  void _initWebSocket(String url, [int retrySeconds = 2]) {
    bool encounteredError = false;

    _ws = new WebSocket(url);

    _ws.onOpen.listen((e) {
      new Timer.periodic(new Duration(milliseconds: 200), (_) {
        var updateStatus = new js.JsObject(js.context['updateStatus'], []);
        String payloadString = '[';
        for (int i = 1; i <= 21; i++) {
          payloadString += containerDiv.children[i].children[1].text;

          if (i == 21) {
            break;
          } else if (i == 4) {
            // Dummy values for extra axes that ROS joy expects.
            payloadString += ',0.0,0.0,0.0,0.0';
          }

          payloadString += i == 4 ? ';' : ',';
        }
        payloadString += ']';

        if (containerDiv.children[1].children[1].text != 'disconnected') {
          _ws.send(payloadString);
        };
      });
    });

    _ws.onError.listen((e) {
      print('Console-$id disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => _initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });
  }

  void _setGamepads() {
    Map deviceIds = js.context['controllers'];
    //deviceIds.sort((a, b) => a.compareTo(b));
    for (int i = 0; i < deviceIds.keys.length; i++) {
      view.addMenuItem({'type': 'toggle', 'title': 'Gamepad$i'}, '#$refName-$id-controllers');
    }
  }

  //\/\/ Mailbox Handlers /\/\//

  void registerMailbox() {

  }

  void registerEventHandlers() {

  }

  Element get elementToFocus => containerDiv;

  Future<bool> preClose() {
    Completer c = new Completer();
    c.complete(true);
    return c.future;
  }

  void cleanUp() {}
}