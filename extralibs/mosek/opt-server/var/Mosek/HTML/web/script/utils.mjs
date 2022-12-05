export function tableElements(n) {
    if (n.nodeName != 'TABLE')
        return undefined;
    else {
        let thead = undefined;
        let tbody = undefined;
        let tfoot = undefined;

        let cn = n.firstElementChild;
        for (; cn ; cn = cn.nextElementSibling) {
            if      (cn.nodeName == 'THEAD')
                thead = cn;
            else if (cn.nodeName == 'TFOOT')
                tfoot = cn;
            else if (cn.nodeName == 'TBODY')
                tbody = cn;
        }

        return { thead:thead, tfoot:tfoot, tbody: tbody };
    }
}

export function util_clearNode(n) {
    if (n) {
	while (n.firstChild) n.firstChild.remove();
    }
}

export function elm_h1(text) {
    var e = document.createElement("h2");
    if (text) e.appendChild(document.createTextNode(text));
    return e;
}

export function elm_h2(text) {
    var e = document.createElement("h2");
    if (text) e.appendChild(document.createTextNode(text));
    return e;
}

export function elm_pre(text,clss) {
    var e = document.createElement("pre");
    if (text) e.appendChild(document.createTextNode(text));
    if (clss) e.setAttribute("class",clss)
    return e;
}

export function elm_div(text,clss) {
    var e = document.createElement("div");
    if (text) e.appendChild(document.createTextNode(text));
    if (clss) e.setAttribute("class",clss)
    return e;
}

export function elm_td(text,clss) {
    var e = document.createElement("td");
    if (text) e.appendChild(document.createTextNode(text));
    if (clss) e.setAttribute("class",clss)
    return e;
}


export function hasClass(n,name) {
    var cls = n.getAttribute("class");
    if (! cls || cls.length == 0)
        return false;
    else {
        var clss = cls.split(/\s+/);
        for (var i in clss) {
            if (clss[i] == name)
                return true;
        }
    }
    return false;
}



export function toggleClass(n,name) {
    var cls = n.getAttribute("class");
    if (! cls || cls.length == 0) {
        n.setAttribute("class",name);
    } else {
        var r = []
        var has = false
        var clss = cls.split(/\s+/);
        for (var i in clss) {
            if (clss[i] == name)
                has = true;
            else
                r[r.length] = clss[i];
        }
        if (! has)
            r[r.length] = name;
        n.setAttribute("class",r.join(" "));
    }
}

export function setClass(n,name,val) {
    var cls = n.getAttribute("class");
    if (! cls || cls.length == 0) {
        if (val)
            n.setAttribute("class",name);
    } else {
        var r = []
        var has = false
        var clss = cls.split(/\s+/);
        for (var i in clss) {
            if (clss[i] == name)
                has = true;
            else
                r.push(clss[i]);
        }
        if (val)
            r.push(name);
        n.setAttribute("class",r.join(" "));
    }
}


export function table_forallVisible(tnode,f) {
    var {tbody} = tableElements(tnode);
    if (tbody) {
        for (var n = tbody.firstElementChild; n; n = n.nextElementSibling)
            if (n.nodeName == "TR" && ! hasClass(n,"hidden"))
                f(n)
    }
}

export function table_forallSelected(tnode,f) {
    var {tbody} = tableElements(tnode);
    if (tbody) {
        for (var n = tbody.firstElementChild; n; n = n.nextElementSibling)
            if (n.nodeName == "TR" && hasClass(n,"selected"))
                f(n)
    }
}
