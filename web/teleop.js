/*
 * axes:
 *
 * 'left stick x axis',
 * 'left stick y axis',
 * 'right stick x axis',
 * 'right stick y axis',
 *
 * buttons:
 *
 * 'A',
 * 'B',
 * 'X',
 * 'Y',
 * 'LB',
 * 'RB',
 * 'LT',
 * 'RT',
 * 'back',
 * 'start',
 * 'L3',
 * 'R3',
 * 'D-U',
 * 'D-D',
 * 'D-L',
 * 'D-R',
 * 'power'
 *
 */

function getStatus() {
    var controllers = {};

    var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
    for (var i = 0; i < gamepads.length; i++) {
        if (!gamepads[i]) { continue; }

        controllers[i] = {"axes": [], "buttons": []};

        var controller = gamepads[i];
        if (controller == null) { continue; }

        for (var k = 0; k < controller.axes.length; k++) {
            if (k == 0 && (controller.axes[1] == 1 || controller.axes[1] == -1)) {
                controllers[i]["axes"].push(0.0);
            } else {
                controllers[i]["axes"].push(-controller.axes[k]);
            }
        }

        for (var l = 0; l < controller.buttons.length; l++) {
            var val = controller.buttons[l];
            var pressed = val == 1.0;
            if (typeof(val) == "object") {
                pressed = val.pressed;
                val = val.value;
            }

            controllers[i]["buttons"].push(pressed ? 1 : 0);
        }
    }

    return JSON.stringify(controllers);
}