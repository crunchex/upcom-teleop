/*
 * Gamepad API Test
 * Written in 2013 by Ted Mielczarek <ted@mielczarek.org>
 *
 * To the extent possible under law, the author(s) have dedicated all copyright and related and neighboring rights to this software to the public domain worldwide. This software is distributed without any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication along with this software. If not, see <http://creativecommons.org/publicdomain/zero/1.0/>.
 */

function getStatus() {
    var controllers = {};

    var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
    for (var i = 0; i < gamepads.length; i++) {
        if (!gamepads[i]) { continue; }

        controllers[i] = [];

        var controller = gamepads[i];
        if (controller == null) { continue; }

        for (var k = 0; k < controller.axes.length; k++) {
            if (k == 0 && (controller.axes[1] == 1 || controller.axes[1] == -1)) {
                controllers[i].push(0.0);
            } else {
                controllers[i].push(-controller.axes[k]);
            }
        }

        for (var l = 0; l < controller.buttons.length; l++) {
            var val = controller.buttons[l];
            var pressed = val == 1.0;
            if (typeof(val) == "object") {
                pressed = val.pressed;
                val = val.value;
            }

            if (pressed) {
                controllers[i].push(1);
            } else {
                controllers[i].push(0);
            }
        }
    }

    return JSON.stringify(controllers);
}