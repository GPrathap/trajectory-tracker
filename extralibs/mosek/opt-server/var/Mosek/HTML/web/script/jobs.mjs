//-*-javascript-*-
import * as optapi from './optapi.mjs';
import * as utils from  './utils.mjs';
import * as gui from    './gui.mjs';

////////////////////////////////////////////////////////////
//
// Search and filter jobs
//
////////////////////////////////////////////////////////////

function matchAny(s,items) {
    for (var i = 0; i < items.length; ++i)
        if (items[i].search(s) >= 0)
            return true;
    return false;
}

function substrMatchAny(substrs,items) {
    for (var i = 0; i < substrs.length; ++i)
        if (! matchAny(substrs[i],items))
            return false;
    return true;
}

var unit_mul_table = {
    "s" : 1, "sec" : 1 , "secs" : 1,  "second" : 1,  "seconds" : 1,
    "m" : 60, "min" : 60, "mins" : 60, "minute" : 60, "minutes" : 60,
    "h" : 3600, "hr" : 3600  , "hrs" : 3600  , "hour" : 3600   , "hours" : 3600,
    "d" : 3600*24, "day" : 3600*24, "days" : 3600*24,
    "M" : 3600*24*30, "month" : 3600*24*30, "months" : 3600*24*30,
    "y" : 3600*24*265, "year" : 3600*24*365, "years" : 3600*24*365
}
function search_buildExp(s) {
    let str = s.trim();
    var regex = /(?:(older|newer)\s*than\s*([0-9]+)\s*([a-zA-Z]+)|(!\s*|not\s+)?(?:"([^"]*)"|'([^']*)'|([^\s]+)))(.*)/;
    //              1                    2          3           4                   5         6        7         8
    var exprs = [];
    var nexprs = [];
    var texpr  = [null,null];
    while (str.length > 0) {
        let m = str.match(regex);
        if (m === null) {
            return null;
        } else {
            var neg = m[4] != null;
            var matchstr = m[5] != null ? m[5] : (m[6] != null ? m[6] : m[7]);
            if      (matchstr) { if (!neg) exprs.push(matchstr); else nexprs.push(matchstr); }
            else if (m[1] != null) {
                var secs = parseInt(m[2]);
                var unit = m[3];
                if (unit in unit_mul_table) {
                    secs *= unit_mul_table[unit];
                } else {
		    return null;
		}

                if      (m[1] == 'older') { if (texpr[1] === null || texpr[1] > secs) texpr[1] = secs; }
                else if (m[1] == 'newer') { if (texpr[0] === null || texpr[0] < secs) texpr[0] = secs; }
            }
            str = m[8].trim();

        }
    }
    return [exprs,nexprs,texpr];
}

// e = [expr,nexpr,texpr]
// expr: substring match any of these produces a match
// nexpr: substring match any of these produces a reject
// texpr: time branket: outside of this rejects
//
// For an expression like "newerthan:4 days !x !y z w"
// Means that the age bracket is [now-4 days; now]
// And the logical test is: !(match x || match y) && (match z || match w)
function exprMatch(e,strs,age) {
    let [expr,nexpr,texpr] = e;

    // test age brancket
    if (age !== NaN) {
        if (( texpr[0] !== null && age > texpr[0] ) ||
            ( texpr[1] !== null && age < texpr[1] ))
            return false;
    }

    if (nexpr.length > 0) {
        // test string neg-matches
        for (var j = 0; j < strs.length; ++j) {
	    for (var i = 0; i < nexpr.length ; ++i) {
                if (strs[j].search(nexpr[i]) >= 0) {
                    return false;
                }
	    }
        }
    }

    if (expr.length == 0)
        return true;
    else {
        // test string matches
        for (var j = 0; j < strs.length; ++j) {
	    for (var i = 0; i < expr.length ; ++i) {
                if (strs[j].search(expr[i]) >= 0) {
                    return true;
                }
	    }
        }
    }

    return false;
}

function exprMatchRow(tr,expr,now) {
    var jobid  = tr.childNodes[1].firstChild.data;
    var desc   = tr.childNodes[2].firstChild ? tr.childNodes[2].firstChild.data : "";
    var owner  = tr.childNodes[3].firstChild.data;
    var date   = Date.parse(tr.childNodes[5].firstChild.data);
    var age;
    if (date === NaN) age = NaN;
    else age = (now-date)/1000;

    return exprMatch(expr,[jobid,desc,owner],age);
}

var cur_input_delay = null;
function jobs_initializeSearchBox() {
    var text  = document.getElementById('inp-search-text');
    var date  = document.getElementById('inp-search-date');
    var tbody = document.getElementById('jobs-table-body');
    var table = document.getElementById('jobs-table');

    document.getElementById("btn-clear-filter").onclick = function() { text.value = ""; text.focus(); };
    document.getElementById("btn-stop-selected").onclick   = function() { utils.table_forallSelected(table,job_stopJobByRow); };
    document.getElementById("btn-delete-selected").onclick = function() { utils.table_forallSelected(table,job_deleteJobByRow); };

    text.oninput = function(ev) {
        if (cur_input_delay != null) clearTimeout(cur_input_delay);
        cur_input_delay = window.setTimeout(function() {
            var str = ev.target.value.trim();
            if (str.length == 0) {
                for (var e = tbody.firstChild; e ; e = e.nextSibling) {
                    utils.setClass(e,"hidden",false);
                }
            } else {
	        var expr = search_buildExp(str);
	        var now = Date.now();
                for (var tr = tbody.firstChild; tr ; tr = tr.nextSibling) {
		    var visible = exprMatchRow(tr,expr,now);
                    utils.setClass(tr,"hidden",!visible);
                }
            }
        }, 1000);
    };
}

function roundN(f,n) {
    var m = (""+f).match("(-?[0-9]*)(?:\.([0-9]*))?(.*)");
    return m[1]+(! m[2] ? "" : ("." +( m[2].length > n ? m[2].substring(0,n) : m[2])))+m[3]
}
function fmtSize(sz) {
    if      (sz < 1024)
        return sz + " B";
    else if (sz < 1024*1024)
        return roundN(sz/1024,2) + " KB";
    else if (sz < 1024*1024*1024)
        return roundN(sz/(1024*1024),2) + " MB";
    else
        return roundN(sz/1024*1024*1024,2) + " GB";
}

function fmtTime(secs) {
    Y = floor(secs/(60*60*24*365))
    val = secs - Y*60*60*24*365;

    M = floor(secs/(60*60*24*30))
    val = secs - M*60*60*24*30;

    d = floor(secs/(60*60*24))
    val = secs - d*60*60*24;

    h = floor(secs/(60*60))
    val = secs - h*60*60;

    m = floor(secs/60)
    val = secs - m*60;

    s = val;

    if (Y > 0 || M > 0 || d > 0)
        return ""+Y+"-"+M+"-"+d+" "+h+":"+m+":"+roundN(s,2);
    else
        return ""+h+":"+m+":"+roundN(s,2);
}

function fmtHrs(secs) {
    let h = Math.floor(secs/(60*60))
    let val = secs - h*60*60;

    let m = Math.floor(val/60)
    val = val - m*60;

    return ""+h+":"+m+":"+roundN(val,2)
}

async function job_startJobByRow(statuscell,tr) {
    var token = tr.getAttribute("data-token");
    try {
        await optapi.startSolve(token);
        if (statuscell !== undefined)
	    tr.childNodes[statuscell].innerHTML = "Running";
    }
    catch (err) {
        // ignore if it fails starting
    }
}

async function job_stopJobByRow(statuscell,tr) {
    var token = tr.getAttribute("data-token");
    await optapi.stopJob(token);
    if (statuscell !== undefined)
	tr.childNodes[statuscell].innerHTML = "Completed";
}

async function job_showDetails(tr) {
    var token = tr.getAttribute("data-token");
    await new Promise((resolve,reject) => {
        gui.activateOverlay(
            async (div) => {
                div.appendChild(utils.elm_h2("Information"));
                var info = utils.elm_div("Loading...");
                div.appendChild(info);
                div.appendChild(utils.elm_h2("Solver log"));
                var log = utils.elm_pre("Loading...");
                div.appendChild(log);
                div.appendChild(utils.elm_h2("Solution"));
                var sol = utils.elm_pre("Loading...");
                div.appendChild(sol);

                let data = await optapi.jobInfo(token);
	        info.innerHTML = "";
                if (data) {
		    var t = document.createElement("table");
		    var tbody = document.createElement("tbody");
		    t.appendChild(tbody);

		    var tr = document.createElement("tr");
		    tr.appendChild(utils.elm_td("Desc:"));
		    tr.appendChild(utils.elm_td(data["Desc"]));
		    tbody.appendChild(tr);

		    var tr = document.createElement("tr");
		    tr.appendChild(utils.elm_td("Owner:"));
		    tr.appendChild(utils.elm_td(data["Ownerid"]));
		    tbody.appendChild(tr);

		    var tr = document.createElement("tr");
		    tr.appendChild(utils.elm_td("Origin Address:"));
		    tr.appendChild(utils.elm_td(data["Submitaddr"]));
		    tbody.appendChild(tr);

		    var tr = document.createElement("tr");
		    tr.appendChild(utils.elm_td("Res:"));
		    tr.appendChild(utils.elm_td(data["ResCode"]));
		    tbody.appendChild(tr);

		    var tr = document.createElement("tr");
		    tr.appendChild(utils.elm_td("Trm:"));
		    tr.appendChild(utils.elm_td(data["TrmCode"]));
		    tbody.appendChild(tr);

		    if (data['Message']) {
		        var tr = document.createElement("tr");
		        tr.appendChild(utils.elm_td("Error message:"));
		        tr.appendChild(utils.elm_td(data["Message"]));
		        tbody.appendChild(tr);
		    }
		    info.appendChild(t);
	        }
                {
                    let data = await optapi.log(token);
                    log.innerHTML = data ? data : "&lt;empty log&gt;";
                }
                {
                    let data = await optapi.solution(token, { "accept" : "text/plain" });
                    sol.innerHTML = data.sol ? data.sol : "&lt;no solution&gt;";
                }
            },
            () => { resolve() });
    });
}

async function job_deleteJobByRow(tr) {
    var token = tr.getAttribute("data-token");
    await optapi.deleteJob(token);
    tr.remove();
}



async function jobs_submitFromFile(file,tbody,cells) {
    let reader = new FileReader();
    let f = await new Promise((resolve,reject) => {
        reader.onload  = resolve;
        reader.onerror = reject;
        reader.onabort = reject;

        reader.readAsBinaryString(file);
    });

    let token = await optapi.submitJob(file.name, f.target.result, { "jobname" : file.name });
    let data  = await optapi.jobInfo(token);
    jobs_addRow(tbody,cells,data,false);
}

function jobs_initializeSubmitBox(tbody,cells) {
    var inpfiles  = document.getElementById("inp-problem-file");
    var btnsubmit = document.getElementById("btn-submit-tasks");
    if (inpfiles && btnsubmit) {
        btnsubmit.onclick = function(ev) {
            var files = inpfiles.files;
            if (files.length > 0) {
                for (var i = 0; i < files.length; ++i) {
                    jobs_submitFromFile(files[i],tbody,cells);
                }
                inpfiles.value = "";
            }
        };
    }
}


function jobs_addRow(tbody,cells,r,atend=true) {
    var tr = document.createElement("tr");
    tr.tabindex = tbody.lastElementChild ? tbody.lastElementChild.tabindex+1 : 1;
    tr.setAttribute("data-token",r.Taskid);
    if (r.Expired)
        utils.setClass(tr,"expired",true);

    var statuscell = undefined;
    for (var j = 0; j < cells.length; ++j) if (cells[j] == 'status') statuscell = j;
    for (var j = 0; j < cells.length; ++j) {
        var td  = document.createElement("td");
        switch (cells[j]) {
        case "id": td.appendChild(document.createTextNode(r.Taskid)); break;
        case "desc": td.appendChild(document.createTextNode(r.Desc)); break;
        case "status": td.appendChild(document.createTextNode(r.Status)); break;
        case "owner": td.appendChild(document.createTextNode(r.Ownerid)); break;
        case "time": td.appendChild(document.createTextNode(r.Submittime)); break;
        case "duration": td.appendChild(document.createTextNode(r.Duration && r.Duration > 0 ? fmtHrs(r.Duration*1e-9) : "")); break;
        case "size": td.appendChild(document.createTextNode(r.Filesize > 0 ? fmtSize(r.Filesize) : undefined, r.Filesize)); break;
        case "ops":
            td.appendChild(
                gui.iconbar_make([
                    gui.icon_make("@info",   "Show info", async () => { return await job_showDetails(tr) }),
                    gui.icon_make("@remove", "Delete job",async () => { return await job_deleteJobByRow(tr) })
                ].concat(r.Status == "Running" ? [ 
                    gui.icon_make("@stop",   "Stop job",  async () => { return await job_stopJobByRow(statuscell,tr) }) ] : [])
                 .concat(r.Status == "Submitted" ? [
                    gui.icon_make("@play",   "Start job", async () => { return await job_startJobByRow(statuscell,tr) }) ] : [])
                ));
            break;
        default:
            break;
        }
        tr.appendChild(td);
    }
    if (atend || ! tbody.firstChild)
        tbody.appendChild(tr);
    else
        tbody.insertBefore(tr,tbody.firstChild);
}


function autodetectFormatFromData(text) {
    // Line comment:
    //   LP: '\'
    //   OPF: '#'
    //   PTF: '#'
    //   MPS: '*'

    let p = text.search(/[^\s]/);
    let t = text;
    //console.log("text = ",t);
    //console.log("  t[0] = ",t[0]);
    switch (t[p]) {
    case '*': return "mps";
    case '\\': return "lp";
    case '[': return "opf";
    case '#': // either ptf or opf
        // skip all line comments
        t = t.substring(p)
        do {
            p = t.search('\n')
            if (p < 0) return undefined;
            t = t.substring(p+1);
            p = text.search(/[^\s]/);
        } while (t[p] == '#');
        // fall through
    default:
        if      (t[p] =='[')           return "opf";
        else if (t.startsWith('Task')) return "ptf";
        else if (t.startsWith('NAME') ||
                 t.startsWith('OBJSENSE') ||
                 t.startsWith('OBJNAME') ||
                 t.startsWith('ROWS')) return "mps";
        else if (t.startsWith('minimize')||
                 t.startsWith('minimum')||
                 t.startsWith('min')||
                 t.startsWith('maximize')||
                 t.startsWith('maximum')||
                 t.startsWith('max'))  return "lp";
        return undefined;
    }
}


async function submitInlineFile(tbody,cells) {
    let node = document.getElementById("inp-problem-inline");
    let text = node.value;

    let format = autodetectFormatFromData(text);
    //console.log("Submit inline: ",format);
    //console.log(text)
    if (format !== undefined) {
        let filename = "problem."+format;
        let token = await optapi.submitJob(filename,text);
        console.log("token = ",token);
        let data  = await optapi.jobInfo(token);
        jobs_addRow(tbody,cells,data,false);
        return token;
    }
}


export async function initialize(user) {
    let table = document.getElementById("jobs-table");
    if (table) {
	jobs_initializeSearchBox();
	gui.table_makeActive(table);
        let global = utils.hasClass(table,"global");
        let {thead,tbody} = utils.tableElements(table);
	let theadrow =  thead.firstElementChild;

	var cells = [];
	var statuscell = undefined;
	if (theadrow)
            for (var n = theadrow.firstElementChild; n ; n = n.nextElementSibling)
		if (n.nodeName == 'TH') {
                    let item = n.getAttribute("data-item")
                    if (item == 'status')
			statuscell = cells.length;
                    cells[cells.length] = item;
		}

        table.jobs_add_row = async (data) => { jobs_addRow(tbody,cells,data) }

	jobs_initializeSubmitBox(tbody,cells);
        gui.table_makeActive(tbody.parentElement);

	if (cells.length > 0 && tbody) {
            let jobs = await optapi.listJobs(global ? undefined : user);
            // sort jobs
            jobs.sort((lhs,rhs)=>{ return lhs.SubmittimeNano > rhs.SubmittimeNano ? -1 : lhs.SubmittimeNano < rhs.SubmittimeNano ? 1 : 0; });

            {
                let nanotimes = [];
                for (var i = 0; i < jobs.length; ++i) { nanotimes.push(jobs[i].SubmittimeNano); }

                console.log("Nanotimes: ",nanotimes);
            }

            // clear and fill table
            while (tbody.firstChild)
		tbody.firstChild.remove();

            for (var i = 0; i < jobs.length; ++i) {
		var r = jobs[i];
		jobs_addRow(tbody,cells,r);
            }
	}

        {
            let node = document.getElementById("btn-submit-inline");
            if (node)
                node.onclick = async ()=>{
                    return await submitInlineFile(tbody,cells)
                };
        }
        {
            let node = document.getElementById("btn-clear-inline");
            if (node)
                node.onclick = ()=>{ document.getElementById("inp-problem-inline").value = "# Paste in your opf, ptf or MPS file here"; };
        }
    }
}

///[TEST]
async function test_jobs() {
    let text = "[comment] Demo problem [/comment]";
    let token = await optapi.submitJob("anonymous.opf", text, { "jobname" : 'anonymous.opf' });
    let data  = await optapi.jobInfo(token);

    var tbody  = document.getElementById("job-tables-body");
    jobs_addRow(tbody,cells,data);
}
