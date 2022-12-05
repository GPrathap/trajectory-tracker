//-*-javascript-*-
import * as optapi from './optapi.mjs';
import * as utils from './utils.mjs';
import * as gui from './gui.mjs';

// Clear New User form
function users_clearForm() {
    document.getElementById('new-user-login').value = "";
    document.getElementById('new-user-name').value  = "";
    document.getElementById('new-user-email').value = "";
    document.getElementById('new-user-pwd1').value  = "";
    document.getElementById('new-user-pwd2').value  = "";
    document.getElementById("new-user-perm-login"        ).checked = false;
    document.getElementById("new-user-perm-submit"       ).checked = false;
    document.getElementById("new-user-perm-use-api"      ).checked = false;
    document.getElementById("new-user-perm-use-tokens"   ).checked = false;
    document.getElementById("new-user-perm-create-tokens").checked = false;
    document.getElementById("new-user-perm-admin"        ).checked = false;
}

// Defines the currently active bubble
var users_current_bubble = null;

// Remove bubble
function users_clearBubble() {
    if (users_current_bubble) {
        users_current_bubble.remove();
        users_current_bubble = null
    }
}

// Create bubble for editing Yes/No field
function users_editYesNo(node, onselect) {
    var curtext = node.firstChild.data;
    var bubble = document.createElement("div");
    bubble.setAttribute("class","bubble");
    var opt_yes = document.createElement("a");
    opt_yes.appendChild(document.createTextNode("YES"));
    bubble.appendChild(opt_yes);
    bubble.appendChild(document.createTextNode("|"));
    var opt_no = document.createElement("a");
    opt_no.appendChild(document.createTextNode("NO"));
    bubble.appendChild(opt_no);

    opt_yes.onclick = function() { users_clearBubble(); if (onselect) { onselect("yes"); } };
    opt_no.onclick  = function() { users_clearBubble(); if (onselect) { onselect("no"); } };

    users_clearBubble();
    node.appendChild(bubble);
    users_current_bubble = bubble;
    opt_yes.focus();
}

// Create bubble for editing one-line text field
function users_editLine(node, text,onselect) {
    var curtext = node.firstChild.data;
    var bubble = document.createElement("div");
    bubble.setAttribute("class","bubble");
    var ed = document.createElement("input");
    ed.setAttribute("type", "text");
    ed.value = text;
    ed.onkeypress = function(ev) {
	if (ev.key == 'Enter') {
	    users_clearBubble();
	    if (onselect) onselect(ed.value);
	}
    };
    bubble.onkeydown = function(ev) { if (ev.key == 'Escape') { users_clearBubble(); } };

    bubble.appendChild(ed);
    users_clearBubble();
    node.appendChild(bubble);
    users_current_bubble = bubble;
    ed.focus();
}

// Create bubble for editing one-line text field
function users_editPassword(node, onselect) {
    var curtext = node.firstChild.data;
    var bubble = document.createElement("div");
    bubble.setAttribute("class","bubble");

    var div1 = document.createElement("div");
    div1.appendChild(document.createTextNode("New password:"))
    bubble.appendChild(div1);

    var ed1 = document.createElement("input");
    ed1.setAttribute("type", "password");
    bubble.appendChild(ed1);

    var div2 = document.createElement("div");
    div2.appendChild(document.createTextNode("Write password again:"))
    bubble.appendChild(div2);

    var ed2 = document.createElement("input");
    ed2.setAttribute("type", "password");
    bubble.appendChild(ed2);

    ed2.onkeypress = function(ev) {
	if (ev.key == 'Enter') {
	    if (ed1.value != ed2.value) {
                ed1.value = "";
                ed2.value = "";
                div1.setAttribute("style","color : red;")
                div2.setAttribute("style","color : red;")
            } else {
	        users_clearBubble();
	        if (onselect) onselect(ed1.value);
            }
	}
    };
    bubble.onkeydown = function(ev) { if (ev.key == 'Escape') { users_clearBubble(); } };
    users_clearBubble();
    node.appendChild(bubble);
    users_current_bubble = bubble;
    ed1.focus();
}

function users_editPermission(userid,permname,permstate) {
    var span = document.createElement("span");
    var input = document.createElement("input");
    input.type = "checkbox";
    input.checked = permstate;
    input.onchange = async () => {
        let perms = {}
        perms[permname] = input.checked;

        let user = await optapi.updateUser(userid, { perms: perms });
        var newstate = user.Permissions[permname];
        if (newstate !== undefined)
            input.checked = newstate
    };
    span.appendChild(input);
    var label = document.createElement("label");
    span.appendChild(label);
    return span;
}

function editEntry(ev) {
    var td = ev.target;
    if (td.nodeName == "TD" && utils.hasClass(td,"editable")) {
        var tr = td.parentNode;
        var userid = tr.firstElementChild.firstChild.data;
        var which = td.getAttribute('data-target');
	var current_value = td.firstChild ? td.firstChild.data : "";
        if (which == "Name") {
	    users_editLine(td,current_value,
                           function(r) {
		               optapi.updateUser(userid,{ name:r })
                                   .then((ok) => { td.innerHTML = r })
                           });
        } else if (which == "Email") {
	    users_editLine(td,current_value, function(r) {
		optapi.updateUser(userid, { email:r })
                    .then(() => { td.innerHTML = r; } );
	    });
        } else if (which == "Password") {
	    users_editPassword(td, function(r) {
		optapi.updateUser(userid, { password:r });
	    });
        } else {
                console.log("WTF?? '"+which+"'")
        }
    }
}


var users_active_edit_cell = null;

async function deleteUser(tr) {
    var userid = tr.getAttribute("data-userid");
    if (userid) {
        await optapi.deleteUser(userid);
        tr.remove();
    }
}


async function createUser() {
    var login = document.getElementById('new-user-login').value;
    if (login && login.length > 0) {
        var name  = document.getElementById('new-user-name').value;
        var email = document.getElementById('new-user-email').value;
        var npwd1  = document.getElementById('new-user-pwd1').value;
        var npwd2  = document.getElementById('new-user-pwd2').value;

        var perms = {};
        perms['login']        = (document.getElementById("new-user-perm-login"        ).checked);
        perms["submit"]       = (document.getElementById("new-user-perm-submit"       ).checked);
        perms["use-api"]      = (document.getElementById("new-user-perm-use-api"      ).checked);
        perms["use-tokens"]   = (document.getElementById("new-user-perm-use-tokens"   ).checked);
        perms["create-tokens"]= (document.getElementById("new-user-perm-create-tokens").checked);
        perms["admin"]        = (document.getElementById("new-user-perm-admin"        ).checked);

        if (npwd1.length > 0 && npwd2.length > 0 && npwd1 != npwd2) {
	    gui.displayError("New password 1 and 2 are not identical - please try again");
            document.getElementById("new-user-pwd1").value = "";
            document.getElementById("new-user-pwd2").value = "";
            document.getElementById("new-user-pwd1").focus();
            return;
        }

        let data = { perms:perms };
        if (name && name.length > 0) data.name = name;
        if (email && email.length > 0) data.email = email;
        if (npwd1 && npwd1.length > 0) data.password = npwd1;

        try {
            let newuser = await optapi.createUser(login,data);
	    users_clearForm();
            await updateUserList();
        }
        catch (err) {
            console.log("err : ",err);
	    gui.displayError("Failed to create user");
            throw err;
        }
    }
}


function addRow(tbody,cells,r) {
    var tr = document.createElement("tr");
    tr.setAttribute("data-userid",r.Userid);
    for (var j = 0; j < cells.length; ++j) {
        var td = document.createElement("td"); tr.appendChild(td);
        if      (cells[j] == "userid")
            td.appendChild(document.createTextNode(r.Userid));
        else if (cells[j] == "name") {
            td.appendChild(document.createTextNode(r.Name));
            td.setAttribute("data-target","Name")
            td.setAttribute("data-type","str")
            td.setAttribute("class","editable")
        }
        else if (cells[j] == "email") {
            td.appendChild(document.createTextNode(r.Email));
            td.setAttribute("data-target","Email")
            td.setAttribute("data-type","str")
            td.setAttribute("class","editable")
        }
        else if (cells[j] == "password") {
            td.appendChild(document.createTextNode("****"));
            td.setAttribute("data-target","Password")
            td.setAttribute("data-type","pwd")
            td.setAttribute("class","editable")
        }
        else if (cells[j] == "perm-login")
            td.appendChild(users_editPermission(r.Userid, "login", r.Permissions["login"]))
        else if (cells[j] == "perm-submit")
            td.appendChild(users_editPermission(r.Userid, "submit", r.Permissions["submit"]))
        else if (cells[j] == "perm-use-token")
            td.appendChild(users_editPermission(r.Userid, "use-tokens", r.Permissions["use-tokens"]))
        else if (cells[j] == "perm-create-token")
            td.appendChild(users_editPermission(r.Userid, "create-tokens", r.Permissions["create-tokens"]))
        else if (cells[j] == "perm-use-api")
            td.appendChild(users_editPermission(r.Userid, "use-api", r.Permissions["use-api"]))
        else if (cells[j] == "perm-admin")
            td.appendChild(users_editPermission(r.Userid, "admin", r.Permissions["admin"]))
        else if (cells[j] == "ops") {
            td.appendChild(
                gui.iconbar_make([
                    gui.icon_make("@remove","Delete User",function(ev) { deleteUser(tr); })
                ]));
        }

        tr.append(td);
        tbody.append(tr);
    }
}

async function updateUserList() {
    let users = await optapi.listUsers();
    var table = document.getElementById("users-table");
    let {thead,tbody,tfoot} = utils.tableElements(table);

    if (thead && tbody) {
        var cells = [];
        if (thead.firstElementChild.nodeName == 'TR')
            for (var n = thead.firstElementChild.firstElementChild; n; n = n.nextElementSibling) {
                if (n.hasAttribute("data-item"))
                    cells[cells.length] = n.getAttribute("data-item");
                else
                    cells[cells.length] = false;
            }
        while (tbody.firstChild) {
            tbody.firstChild.remove();
        }

        for (var i in users)
	    addRow(tbody,cells,users[i]);

        tbody.onclick = editEntry;
    }
}

// Call to initialize the Users View page, populate lists etc.
export async function initialize() {
    document.getElementById('btn-create-user').onclick = createUser;
    await updateUserList();
}
