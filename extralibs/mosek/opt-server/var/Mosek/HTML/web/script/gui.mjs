//-*-javascript-*-
import * as optapi from './optapi.mjs';
import * as utils from './utils.mjs';


//////////////////////////////////////////////////////////////////////
//
// DYNAMIC GUI FUNCTIONS
//
//////////////////////////////////////////////////////////////////////

export function displayError(msg) { document.getElementById("error-msg").innerHTML  = msg; }
export function clearError() { document.getElementById("error-msg").innerHTML  = ""; }

export function displayMessage(msg) { document.getElementById("status-msg").innerHTML  = msg; }
export function clearMessage() { document.getElementById("status-msg").innerHTML  = ""; }

async function gui_logout() {
    return await optapi.logout().then(
        (ok)  => { window.location = "/static/login.html?origin="+window.location.pathname; },
        (err) => {});
}

var identity = null;

function nav_makeEntry(name,href) {
    var li = document.createElement('li');
    var a  = document.createElement('a');
    a.appendChild(document.createTextNode(name))
    a.setAttribute("href",href);

    li.appendChild(a);
    return li;
}

function menu_makeEntry(name,href) {
    var li = document.createElement('li');
    var a  = document.createElement('a');
    a.appendChild(document.createTextNode(name))
    a.setAttribute("href",href);
    li.appendChild(a);
    return li;
}






export function activateOverlay(populate,onoverlayclose) {
    var overlay = document.getElementById("overlay");
    utils.setClass(overlay,"hidden",false);
    if (overlay) {
        var content = overlay.firstElementChild;
        while (content.firstElementChild) content.firstElementChild.remove();
	var div = document.createElement("div");
	div.setAttribute("class","button button-close");
	div.setAttribute("style","float : right;")
	content.appendChild(div);
        overlay.onclick = function() {
            utils.setClass(overlay,"hidden",true);
            if (onoverlayclose)
                onoverlayclose();
        };
        populate(content);
    }
}



export function table_makeActive(tnode) {
    var {thead,tbody} = utils.tableElements(tnode);

    if (thead && tbody) {
        var cn = thead.firstElementChild;
        if (cn && cn.nodeName == 'TR') {
            cn = cn.firstElementChild;
            if (cn && cn.nodeName == 'TH') {
                cn = cn.firstElementChild;
                if (cn && cn.nodeName == 'INPUT') {
                    cn.onchange = function(ev) {
                        var check = ev.target.checked;
                        for (var tr = tbody.firstElementChild; tr ; tr = tr.nextElementSibling) {
                            if (tr.nodeName == "TR" && ! utils.hasClass(tr,"hidden"))
                                utils.setClass(tr,"selected",check);
                        }
                    };
                }
            }
        }

        tbody.onclick = function(ev) {
            var n = ev.target;
            if (n.nodeName == "TD") n = n.parentElement;
            if (n.nodeName == "TR") utils.toggleClass(n,"selected")
        }
    }
}


function table_populate(t,data,addrow_func) {
    var tbody = t.firstElementChild;
    while (tbody && tbody.nodeName != "TBODY") tbody = tbody.nextElementSibling;

    if (tbody.nodeName == 'TBODY') {
        while (tbody.firstChild)
            tbody.firstChild.remove();
        for (var i = 0; i < data.length; ++i)
            addrow_func(tbody,data[i])
    }
}

export function iconbar_make(icons,orientation) {
    var ul = document.createElement("ul"); ul.setAttribute("class","icons")
    if (orientation == "vertical")
        strClass(ul,"vertical",true);
    else if (orientation == "horizontal")
        strClass(ul,"horizontal",true);
    for (var i = 0; i < icons.length; ++i)
        ul.appendChild(icons[i]);
    return ul;
}

export function icon_make(name,label,onclick) {
    var li = document.createElement("li");
    var a  = document.createElement("a"); li.appendChild(a);
    a.onclick = onclick;
    utils.setClass(a,"icon",true);
    if (name.startsWith("@"))
	utils.setClass(a,"fa-"+name.substring(1),true);
    else
        utils.setClass(a,name,true);

    var span = document.createElement("span"); a.appendChild(span);
    span.setAttribute("class","label");
    span.appendChild(document.createTextNode(label));

    return li;
}

export function button_make(label,onclick) {
    var btn = document.createElement("div");
    if (label && label.length > 0 && label[0] != '@')
        btn.appendChild(document.createTextNode(label));

    utils.setClass(btn,"button",true);
    if (label && label.length > 0 && label[0] == '@')
        utils.setClass(btn,"button-"+label.substring(1),true);
    btn.onclick = onclick;
    return btn;
}


function element_makeTD(text,data) {
    var td = document.createElement("td");
    if (data !== undefined)
        td.setAttribute("data",data);
    if (text !== undefined)
        td.appendChild(document.createTextNode(text));
    return td;
}


export async function initialize() {
    let user = await optapi.getUser();
    if (user === undefined) {
        window.location = "/static/login.html?origin="+window.location.pathname;
        return;
    }

    /*global*/ identity = user;

    var e = document.getElementById('user-id'); if (e) e.innerHTML = identity.Userid;
    var nav = document.getElementById('nav-items');
    var usermenu = document.getElementById('user-items');
    if (nav)
	if (user.Permissions["admin"]) {
            nav.insertBefore(menu_makeEntry("Stats","stats.html"),nav.firstChild);
            nav.insertBefore(menu_makeEntry("Jobs","alljobs.html"),nav.firstChild);
            nav.insertBefore(menu_makeEntry("Access Tokens","alltokens.html"),nav.firstChild);
            nav.insertBefore(menu_makeEntry("Users","userlist.html"),nav.firstChild);
	}

    //if (onready != null) {
    //    onready(user);
    //}

    var main = document.getElementById("main");
    if (main) {
        var overlay = document.createElement("div");
        overlay.setAttribute("id","overlay");
        overlay.setAttribute("class","hidden");

        var content = document.createElement("div");
        content.setAttribute("class","overlay-content");
        overlay.appendChild(content);

        main.parentElement.appendChild(overlay);
    }

    // if (document.getElementById("tokens-table"))
    //     await initialize_tokensTable(user);
    // if (document.getElementById("users-table"))
    //     await initialize_usersTable(user);
    // if (document.getElementById("jobs-table"))
    //     await initialize_jobsTable(user);
    // if (document.getElementById("user-form"))
    //     await profile_fillInDetails(user);

    var ns = document.getElementsByClassName("select-user-list");
    if (ns.length > 0) {
        let data = await optapi.listUsers();
        for (var i = 0; i < ns.length; ++i) {
            var n = ns[i];
            for (var j = 0; j < data.length; ++j) {
                var opt = document.createElement("option");
                opt.setAttribute("value", data[j].Userid);
                opt.appendChild(document.createTextNode(data[j].Userid));
                n.appendChild(opt);
            }
        }
    }

    {
        let node = document.getElementById("btn-logout");
        if (node !== undefined) {
            node.onclick = gui_logout
        }
    }

    return user;
}
