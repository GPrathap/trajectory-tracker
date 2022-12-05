//-*-javascript-*-
import * as optapi from './optapi.mjs';

async function login_loginAndReload() {
    var username = document.getElementById('inp-username').value;
    var password = document.getElementById('inp-password').value;

    try {
        let user = await optapi.login(username,password);

        var href = window.location.href;
        var p = href.indexOf('?');
        if (p < 0) {
            //console.log('window.location = "/web/index.html"');
            window.location = "/web/index.html";
        } else {
            var fragments = href.substring(p+1,href.length).split('&');
            for (var i in fragments) {
                if (fragments[i].startsWith("origin=")) {
                    //console.log('window.location = '+fragments[i].substring(7));
                    window.location = fragments[i].substring(7);
                    break;
                }
            }
        }
    }
    catch (err) {
        document.getElementById('msg-error').style.display = 'inline';
        var e = document.getElementById('inp-password');
        e.focus();
        e.select();
        throw err;
    }
}

export function initialize() {
    //console.log("login.initialize");
    document.getElementById("btn-submit").onclick = login_loginAndReload;
    document.getElementById("inp-password").onkeypress = function(ev) { if (ev.key == "Enter") login_loginAndReload(); };
    document.getElementById("inp-username").focus();
}
