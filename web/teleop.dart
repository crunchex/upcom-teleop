library updroid_teleop;

import 'dart:html';
import 'dart:async';
import 'dart:js' as js;

import 'package:upcom-api/tab_frontend.dart';

class UpDroidTeleop extends TabController {
  static final List<String> names = ['upcom-teleop', 'UpDroid Teleop', 'Teleop'];

  static List getMenuConfig() {
    List menu = [
      {'title': 'File', 'items': [
        {'type': 'toggle', 'title': 'Close Tab'}]},
      {'title': 'View', 'items': [
        {'type': 'toggle', 'title': 'Swap Cameras'}]},
      {'title': 'Controllers', 'items': []}
    ];
    return menu;
  }

  DivElement containerDiv, _toolbar;

  WebSocket _ws;
  var _mainStream, _thumbnailStream;
  Timer _resizeTimer;
  SpanElement _gamepadButton, _keyboardButton;
  String _leftImageSrc, _rightImageSrc, _mainImageSrc, _thumbnailImageSrc;
  AnchorElement _swapCamerasButton;

  // Use a pre-recorded video file instead of livestreams.
  // FOR DEVELOPMENT ONLY.
  bool _demoMode = false;

  UpDroidTeleop() :
  super(UpDroidTeleop.names, getMenuConfig(), 'tabs/upcom-teleop/teleop.css') {
    String ip = window.location.host.split(':')[0];

    if (_demoMode) {
      _leftImageSrc = 'tabs/upcom-teleop/pov-l.m4v';
      _rightImageSrc = 'tabs/upcom-teleop/pov-r.m4v';
    } else {
      _leftImageSrc = 'http://10.4.0.215:12062/stream?topic=/stereo/left/image_raw';
      _rightImageSrc = 'http://10.4.0.215:12062/stream?topic=/stereo/right/image_raw';
    }
  }

  void setUpController() {
    // Dummy div so that teleop fits with the onFocus API from tab controller.
    containerDiv = new DivElement()
      ..id = '$refName-$id-container'
      ..classes.add('$refName-container');
    view.content.children.add(containerDiv);

    _swapCamerasButton = view.refMap['swap-cameras'];

    if (_demoMode) _initTeleop(new Msg('DUMMY'));

//    DivElement keyboardDiv = new DivElement()
//      ..classes.add('$refName-keyboard');
//    containerDiv.children.add(keyboardDiv);
//
//    // Set up the keyboard overlay.
//    ImageElement keyboardBackground = new ImageElement(src: 'tabs/$refName/87_keyboard_bg.jpg')
//      ..classes.add('$refName-keyboard-bg');
//    keyboardDiv.children.add(keyboardBackground);
//
//    ImageElement keyboardBase = new ImageElement(src: 'tabs/$refName/87-keybase.png')
//      ..classes.add('$refName-keyboard-keys');
//    keyboardDiv.children.add(keyboardBase);
//
//    ImageElement keyboardMods = new ImageElement(src: 'tabs/$refName/87-mods-modern.png')
//      ..classes.add('$refName-keyboard-keys');
//    keyboardDiv.children.add(keyboardMods);
//
//    ImageElement keyboardKeys = new ImageElement(src: 'tabs/$refName/87-modern.png')
//      ..classes.add('$refName-keyboard-keys');
//    keyboardDiv.children.add(keyboardKeys);

    // Set up the toolbar.
    _toolbar = new DivElement()
      ..classes.add('toolbar');
    view.content.children.add(_toolbar);

    _gamepadButton = new SpanElement()
      ..title = 'Gamepad Control'
      ..classes.addAll(['glyphicons', 'glyphicons-gamepad']);
    _keyboardButton = new SpanElement()
      ..title = 'Keyboard Control'
      ..classes.addAll(['glyphicons', 'glyphicons-keyboard-wireless']);

    _toolbar.children.addAll([_keyboardButton, _gamepadButton]);
  }

  void _setMainFeed(String src) {
    if (_demoMode) {
      _mainStream = new VideoElement()
        ..id = '$refName-$id-main-video'
        ..classes.add('$refName-video')
        ..loop = true
        ..autoplay = true;

      SourceElement mainVideoSource = new SourceElement()
        ..src = src;
      _mainStream.children.add(mainVideoSource);
    } else {
      _mainStream = new ImageElement(src: src)
        ..id = '$refName-$id-main-stream'
        ..classes.add('$refName-stream');
    }

    containerDiv.children.add(_mainStream);

    // Timer to let the streams settle.
    new Timer(new Duration(milliseconds: 500), () {
      _setStreamDimensions();
    });

    _mainImageSrc = src;
  }

  void _setThumbnailFeed(src) {
    if (_demoMode) {
      _thumbnailStream = new VideoElement()
        ..id = '$refName-$id-thumbnail-video'
        ..classes.addAll(['$refName-video', 'small'])
        ..loop = true
        ..autoplay = true;

      SourceElement thumbnailVideoSource = new SourceElement()
        ..src = src;
      _thumbnailStream.children.add(thumbnailVideoSource);
    } else {
      _thumbnailStream = new ImageElement(src: src)
        ..id = '$refName-$id-thumbnail-stream'
        ..classes.addAll(['$refName-stream', 'small']);
    }

    containerDiv.children.add(_thumbnailStream);

    _thumbnailImageSrc = src;
  }

  void _setUpControl() {
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

    _setGamepads();
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
          print('payload: $payloadString');
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

  void _initTeleop(Msg m) {
    _setMainFeed(_leftImageSrc);
    _setThumbnailFeed(_rightImageSrc);
//    _setUpControl();
  }

  void registerMailbox() {
    if (!_demoMode) mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'NODES_UP', _initTeleop);
  }

  void _swapImageFeeds() {
    _mainStream.remove();
    _thumbnailStream.remove();

    _setMainFeed(_mainImageSrc == _leftImageSrc ? _rightImageSrc : _leftImageSrc);
    _setThumbnailFeed(_thumbnailImageSrc == _leftImageSrc ? _rightImageSrc : _leftImageSrc);
  }

  void _setStreamDimensions() {
    if (containerDiv.contentEdge.width < containerDiv.contentEdge.height) {
      // Usually normal mode.
      _mainStream.style.width = '100%';
      String newHeight = '${(containerDiv.contentEdge.width * 240 / 320).toString()}px';
      _mainStream.style.height = newHeight;

      double margin = (containerDiv.contentEdge.height - _mainStream.contentEdge.height) / 2;
      _mainStream.style.margin = '${margin.toString()}px 0 ${margin.toString()}px 0';
    } else {
      // Usually maximized mode.
      _mainStream.style.height = 'calc(100% - 32px)';
      String newWidth = '${(containerDiv.contentEdge.height * 320 / 240).toString()}px';
      _mainStream.style.width = newWidth;

      double margin = (containerDiv.contentEdge.width - _mainStream.contentEdge.width) / 2;
      _mainStream.style.margin = '0 ${margin.toString()}px 0 ${margin.toString()}px';
    }
  }

  void registerEventHandlers() {
    _swapCamerasButton.onClick.listen((e) => _swapImageFeeds());

    window.onResize.listen((e) {
      if (_resizeTimer != null) _resizeTimer.cancel();
      _resizeTimer = new Timer(new Duration(milliseconds: 500), () {
        _setStreamDimensions();
      });
    });
  }

  Element get elementToFocus => containerDiv;

  Future<bool> preClose() {
    Completer c = new Completer();
    c.complete(true);
    return c.future;
  }

  void cleanUp() {}
}