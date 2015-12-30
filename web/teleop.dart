library updroid_teleop;

import 'dart:html';
import 'dart:async';
import 'dart:js';
import 'dart:convert';
import 'dart:collection' show Maps;

import 'package:upcom-api/web/mailbox/mailbox.dart';
import 'package:upcom-api/web/tab/tab_controller.dart';
import 'package:upcom-api/web/menu/plugin_menu.dart';

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

  DivElement containerDiv, _toolbar, _mainStreamDiv, _thumbnailStreamDiv;

  WebSocket _ws;
  var _mainStream, _thumbnailStream;
  ParagraphElement _mainStreamLabel, _thumbnailStreamLabel;
  Timer _resizeTimer;
  SpanElement _gamepadButton, _keyboardButton;
  String _leftImageSrc, _rightImageSrc, _mainImageSrc, _thumbnailImageSrc;
  AnchorElement _swapCamerasButton;
  ScriptElement _teleopJs;

  // Use a pre-recorded video file instead of livestreams.
  // FOR DEVELOPMENT ONLY.
  bool _demoMode = true;

  UpDroidTeleop(ScriptElement script) :
  super(UpDroidTeleop.names, true, true, getMenuConfig()) {
    _teleopJs = script;

    String ip = window.location.host.split(':')[0];

    if (_demoMode) {
      _leftImageSrc = 'plugins/upcom-teleop/pov-l.m4v';
      _rightImageSrc = 'plugins/upcom-teleop/pov-r.m4v';
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
    content.children.add(containerDiv);

    _mainStreamDiv = new DivElement();
    containerDiv.children.add(_mainStreamDiv);

    _thumbnailStreamDiv = new DivElement();
    containerDiv.children.add(_thumbnailStreamDiv);

    _mainStreamLabel = new ParagraphElement()
      ..id = '$refName-$id-main-label'
      ..classes.add('$refName-main-label')
      ..text = 'Left Cam';
    _mainStreamDiv.children.add(_mainStreamLabel);

    _thumbnailStreamLabel = new ParagraphElement()
      ..id = '$refName-$id-thumbnail-label'
      ..classes.add('$refName-thumbnail-label')
      ..text = 'Right Cam';
    _thumbnailStreamDiv.children.add(_thumbnailStreamLabel);

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
    content.children.add(_toolbar);

    _gamepadButton = new SpanElement()
      ..title = 'Gamepad Control'
      ..classes.addAll(['glyphicons', 'glyphicons-gamepad']);
    _keyboardButton = new SpanElement()
      ..title = 'Keyboard Control'
      ..classes.addAll(['glyphicons', 'glyphicons-keyboard-wireless']);

    _toolbar.children.addAll([_keyboardButton, _gamepadButton]);

    _swapCamerasButton = refMap['Swap Cameras'];
  }

  void _setMainFeed(String src) {
    if (_demoMode) {
      _mainStream = new VideoElement()
        ..id = '$refName-$id-main-video'
        ..loop = true
        ..autoplay = true;

      SourceElement mainVideoSource = new SourceElement()
        ..src = src;
      _mainStream.children.add(mainVideoSource);

      _mainStreamDiv.classes.add('$refName-video');
    } else {
      _mainStream = new ImageElement(src: src)
        ..id = '$refName-$id-main-stream';

      _mainStreamDiv.classes.add('$refName-stream');
    }

    _mainStream.classes.add('$refName-main');
    _mainStreamDiv.children.add(_mainStream);

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
        ..loop = true
        ..autoplay = true;

      SourceElement thumbnailVideoSource = new SourceElement()
        ..src = src;
      _thumbnailStream.children.add(thumbnailVideoSource);

      _thumbnailStreamDiv.classes.addAll(['$refName-video', 'small']);
    } else {
      _thumbnailStream = new ImageElement(src: src)
        ..id = '$refName-$id-thumbnail-stream';

      _thumbnailStreamDiv.classes.addAll(['$refName-stream', 'small']);
    }

    _thumbnailStream.classes.add('$refName-thumbnail');
    _thumbnailStreamDiv.children.add(_thumbnailStream);

    _thumbnailImageSrc = src;
  }

  void _setUpControl() {
    String url = window.location.host;
    url = url.split(':')[0];
    // window.location.host returns whatever is in the URL bar (including port).
    // Since the port here needs to be dynamic, the default needs to be replaced.
    _initWebSocket('ws://' + url + ':12060/$refName/$id/controller/0');

//    _setGamepads();
  }

  void _initWebSocket(String url, [int retrySeconds = 2]) {
    bool encounteredError = false;

    _ws = new WebSocket(url);

    _ws.onOpen.listen((e) {
      new Timer.periodic(new Duration(milliseconds: 200), (_) {
        Map controllerStatus = JSON.decode(context.callMethod('getStatus').toString());
        // Only handling one controller for now.
        _ws.send(controllerStatus['0']);
      });
    });

    _ws.onError.listen((e) {
      print('Teleop-$id disconnected. Retrying...');
      if (!encounteredError) {
        new Timer(new Duration(seconds:retrySeconds), () => _initWebSocket(url, retrySeconds * 2));
      }
      encounteredError = true;
    });
  }

  void _setGamepads() {
    Map deviceIds = context['controllers'];
    //deviceIds.sort((a, b) => a.compareTo(b));
    for (int i = 0; i < deviceIds.keys.length; i++) {
      addMenuItem(id, refName, {'type': 'toggle', 'title': 'Gamepad$i'}, refMap, '#$refName-$id-controllers');
    }
  }

  void _initTeleop(Msg m) {
    _setMainFeed(_leftImageSrc);
    _setThumbnailFeed(_rightImageSrc);
    _setUpControl();
  }

  void registerMailbox() {
    mailbox.registerWebSocketEvent(EventType.ON_MESSAGE, 'NODES_UP', _initTeleop);
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
      _mainStreamDiv.style.width = '100%';
      String newHeight = '${(containerDiv.contentEdge.width * 240 / 320).toString()}px';
      _mainStreamDiv.style.height = newHeight;

      double margin = (containerDiv.contentEdge.height - _mainStreamDiv.contentEdge.height) / 2;
      _mainStreamDiv.style.margin = '${margin.toString()}px 0 ${margin.toString()}px 0';
    } else {
      // Usually maximized mode.
      _mainStreamDiv.style.height = 'calc(100% - 32px)';
      String newWidth = '${(containerDiv.contentEdge.height * 320 / 240).toString()}px';
      _mainStreamDiv.style.width = newWidth;

      double margin = (containerDiv.contentEdge.width - _mainStreamDiv.contentEdge.width) / 2;
      _mainStreamDiv.style.margin = '0 ${margin.toString()}px 0 ${margin.toString()}px';
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

  void cleanUp() {
    _teleopJs.remove();
  }
}

//class JsMap implements Map<String,dynamic> {
//  final JsObject _jsObject;
//  JsMap.fromJsObject(this._jsObject);
//
//  operator [](String key) => _jsObject[key];
//  void operator []=(String key, value) {
//    _jsObject[key] = value;
//  }
//  remove(String key) {
//    final value = this[key];
//    _jsObject.deleteProperty(key);
//    return value;
//  }
//  Iterable<String> get keys => context['Object'].callMethod('keys', [_jsObject]);
//
//  // use Maps to implement functions
//  bool containsValue(value) => Maps.containsValue(this, value);
//  bool containsKey(String key) => keys.contains(key);
//  putIfAbsent(String key, ifAbsent()) => Maps.putIfAbsent(this, key, ifAbsent);
//  void addAll(Map<String, dynamic> other) {
//    if (other != null) {
//      other.forEach((k,v) => this[k] = v);
//    }
//  }
//  void clear() => Maps.clear(this);
//  void forEach(void f(String key, value)) => Maps.forEach(this, f);
//  Iterable get values => Maps.getValues(this);
//  int get length => Maps.length(this);
//  bool get isEmpty => Maps.isEmpty(this);
//  bool get isNotEmpty => Maps.isNotEmpty(this);
//}