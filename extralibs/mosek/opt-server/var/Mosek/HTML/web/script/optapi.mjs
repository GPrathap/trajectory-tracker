//-*-javascript-*-
import * as req from './request.mjs';

async function raw_POST(url,data,kwds = {}) {
    kwds['data'] = data;
    return req.request(url,'POST',kwds);
}

async function POST(url,senddata,kwds={}) { //
    let {status,data} = await raw_POST(url,senddata,kwds);
    return (status == 200 ? data : undefined);
}

async function raw_GET(url,kwds={}) {
    return req.request(url,'GET',kwds);
}

async function raw_HEAD(url,kwds={}) {
    return req.request(url,'HEAD',kwds);
}

async function GET(url,kwds={}) {
    let {status,data} = await raw_GET(url,kwds);
    return status == 200 ? data : undefined;
}

function getHeaderValue(name,hdrs) {
    let p = hdrs.search(new RegExp(`^${name}:`,'m'));
    if (p < 0)
        p = hdrs.search(new RegExp(`^${name.toLowerCase()}:`,'m'));
    if (p < 0)
        return undefined;

    let pv = hdrs.indexOf(':',p);
    if (pv < 0) return undefined;
    let pe = hdrs.indexOf('\n',pv);
    if (pe < 0) return undefined;

    return hdrs.substring(pv+1,pe).trim();
}

function parseHeaders(hdrs) {
    let lines = hdrs.split('\r\n');
    let res = {};
    for (let i = 0; i < hdrs.length; ++i) {
        let p = lines[i].indexOf(':');
        if (p > 0) {
            let k = lines[i].substring(0,p);
            let v = lines[i].substring(p+1).trim();
            if (k in res) res[k].push(v);
            else res[k] = [v];
        }
    }
    return res;
}


////////////////////////////////////////////////////////////
//
// LOGIN / LOGOUT
//
////////////////////////////////////////////////////////////

export async function getUser({host} = {}) {
    let url = "/users/api/whoami";
    if (host !== undefined) url = host+url;
    let data = await GET(url);
    return data === undefined ? undefined : JSON.parse(data);
}

export async function login(username,password,{host}={}) {
    let  url = "/users/api/login";
    if (host) url = host+url;

    let data = JSON.stringify({"Username":username,"Password":password});
    return JSON.parse(await POST(url,
                                 data,
                                 {contenttype:"application/json"}));
}

export async function logout({host}={}) {
    var url = "/users/api/logout";
    if (host) url = host+url;
    return await GET(url);
}

////////////////////////////////////////////////////////////
//
// ACCESS TOKENS
//
////////////////////////////////////////////////////////////

// data: { Userid : string,
//         Permissions : optional list([name : string, onoff : bool]),
//         Description : optional string }
// onreply: function(data : TokenData)
//    TokenData : {
//       Serial      : int
//       Name        : string
//       Owner       : string
//       Expires     : int
//       Description : string
//       Permissions : map( permname : string -> bool )
//    }
export async function createToken({owner,host,perms,expires,desc}={}) {
    var url = "/users/api/token/create";
    if (host) url = host+url;

    let data = { };
    if (owner !== undefined) { data.Owner = owner; }
    if (perms !== undefined) { data.Permissions = perms; }
    if (desc  !== undefined) { data.Description = desc;  }
    if (expires  !== undefined) { data.Expires = expires; }

    return JSON.parse(await POST(url,
                                 JSON.stringify(data),
                                 { headers:{'Content-Type' : "application/json"} }));
}

// serial : int
// onreply : function(ok : bool)
export async function deleteToken(serial,{host}={}) {
    var url = "/users/api/token/delete";
    if (host) url = host+url;

    return await GET(url,{query: {"token-serial":serial} });
}

// onreply: function(ok : bool, data : list(TokenData))
//    TokenData : {
//       Serial       : int,
//       Owner        : string,
//       Expires      : int,
//       Permissions  : map( permname : string -> bool ),
//       Description  : string
//    }
// user : undefined | username
export async function listTokens({user,host}={}) {
    var url = "/users/api/token/list";
    if (host) url = host+url;
    let query;

    let data = {};
    if (user !== undefined) data['Owner'] = user;
    return JSON.parse(await POST(url,JSON.stringify(data)));
}

////////////////////////////////////////////////////////////
//
// JOBS
//
////////////////////////////////////////////////////////////

export async function listJobs({host,user}={}) {
    var url = "/users/api/jobs/list";
    if (host) url = host+url;

    return (user === undefined ?
            JSON.parse(await GET(url)) :
            JSON.parse(await POST(url,
                                  JSON.stringify({Owner:user}),
                                  {headers:{'Content-Type' : "application/json"}})));
}

export async function startJob(token, {host}={}) {
    let url = "/api/solve";
    if (host) url = host+url;

    return await GET(url,{query:{token:token}});
}

export async function deleteJobs(tokens,{host}={}) {
    let url = "/users/api/jobs/delete";
    if (host) url = host+url;
    var data = [];
    for (var i = 0; i < tokens.length; ++i) data.push( String(tokens[i]));
    return await POST(url,data,{headers:{'Content-Type' : "application/json"}});
}

export async function jobInfo(token,{host}={}) {
    let url = "/users/api/jobs/info";
    if (host) url = host+url;
    return JSON.parse(await GET(url,{query:{token:token}}));
}

////////////////////////////////////////////////////////////
//
// USERS
//
////////////////////////////////////////////////////////////

export async function createUser(user,{host,name,email,password,perms}={}) {
    var url = "/users/api/user/create";
    if (host) url = host+url;

    var data = { Userid:user };
    if (name !== undefined) data.Name = name;
    if (email !== undefined) data.Email = email;
    if (password !== undefined) data.Password = password;
    if (perms !== undefined) {
        var ps = [];
        for (var p in perms) {
            if (p == "admin"  ||
                p == "submit" ||
                p == "login"  ||
                p == "use-tokens" ||
                p == "create-tokens" ||
                p == "use-api")
                ps.push([p,perms[p]]);
            else
                throw new Error("Invalid permission");
        }

        data.Permissions = ps;
    }
    return JSON.parse(
        await POST(url,
                   JSON.stringify(data),
                   {headers:{'Content-Type' : "application/json"}}));
}


export async function deleteUser(user,{host}={}) {
    let url = "/users/api/user/delete";
    if (host) url = host+url;
    return await POST(url,JSON.stringify({Userid:user}));
}

export async function updateUser(user,{host,name,email,password,oldpassword,perms}={}) {
    let data = { Userid:user };
    if (name     !== undefined) data.Name = name;
    if (email    !== undefined) data.Email = email;
    if (password !== undefined) data.Password = password;
    if (oldpassword !== undefined) data.Oldpassword = oldpassword;
    if (perms    !== undefined) {
        let ps = [];
        for (var p in perms) {
            if (p == "admin" ||
                p == "submit" ||
                p == "login" ||
                p == "use-tokens" ||
                p == "create-tokens" ||
                p == "use-api")
                ps.push([p,perms[p]]);
            else
                throw new Error("Invalid permission"+p);
        }
        data.Permissions = ps;
    }

    let url = "/users/api/user/update";
    if (host) url = host+url;

    //console.log("password = ",oldpassword);
    return JSON.parse(await POST(url,JSON.stringify(data),{headers:{'Content-Type' : "application/json"}}))
}

export async function listUsers({host}={}) {
    var url = "/users/api/user/list";
    if (host) url = host+url;

    return JSON.parse(await GET(url));
}


////////////////////////////////////////////////////////////
//
// Basic API
//
////////////////////////////////////////////////////////////


// filename    : string
// filedata    : string or blob that can be written
// onrepy      : optional function(jobtoken:string)
// data        : optional dictionary
//    The dictionary defines additional options
//    - "access-token" is an access token to use


export async function submitJob(filename,filedata,{host,jobname,accesstoken}={}) {
    var contentType = "application/binary";
    var ext_start   = filename.lastIndexOf('.');
    var ext_start2  = -1;
    var ext = ext_start >= 0 ? filename.substring(ext_start) : "";
    var ext2 = "";
    if (ext == "bz2" || ext == "gz" || ext == "zst") {
        ext2 = ext;
        ext_start2 = ext_start;
        ext_start = filename.lastIndexIf('.',ext_start2-1);
        ext = ext_start >= 0 ? filename.substring(ext_start,ext_start2) : "";
    }

    if (ext == ".lp"    ||
        ext == ".mps"   ||
        ext == ".fmps"  ||
        ext == ".opf"   ||
        ext == ".cbf"   ||
        ext == ".ptf"   ||
        ext == ".task"  ||
        ext == ".jtask") {
        contentType = "application/x-mosek-"+ext.substring(1);
    } else {
        contentType = "application/x-mosek-auto";
    }

    if      (ext2 == ".bz2") contentType += "+bzip2";
    else if (ext2 == ".gz")  contentType += "+gzip";
    else if (ext2 == ".zst") contentType += "+zstd";


    var url = "/api/submit";
    if (host) url = host+url;

    let query = {};
    if (jobname !== undefined)
        query.jobname = jobname;

    let headers = { 'Content-Type' : contentType };
    if (accesstoken !== undefined) headers["X-Mosek-Access-Token"] = accesstoken;


    return await POST(url,filedata,
                          {contenttype:contentType,
                           headers:headers,
                           query:query});
}

// jobtoken : string
// onreply  : optional function(ok : bool)
// data        : optional dictionary
//    The dictionary defines additional options
//    - "access-token" is an access token to use
export async function startSolve(jobtoken,{host,accesstoken}={}) {
    var url = "/api/solve-background";
    if (host) url = host+url;

    let headers = {};
    if (accesstoken !== undefined)
        headers["X-Mosek-Access-Token"] = accesstoken;

    return await GET(url,{query:{token:jobtoken},headers:headers});
}

// jobtoken : string
// onreply  : optional function(solution  : optional string,
//                              solformat : optional string,
//                              res       : optional string,
//                              trm       : optional string,
//                              msg       : optional string)
//    Where:
//       - solution is the solution file as a string.
//       - solformat is the MIME type if the solution
//       - res is the solver response code
//       - trm is the solver termination code
//       - msg is the error message of res!=MSK_RES_OK
//    If the solver fails completely, solution and solformat are
//    undefined, and res,trm and msg may or may not be defined,
//    otherwise solution and solformat are defined.
// data        : optional dictionary
//    The dictionary defines additional options
//    - "access-token" is an access token to use
//    - "accept" Requested MIME Type of result, "application/x-mosek-task",
//      "application/x-mosek-jtask" or "text/plain"
export async function solveJobAndWait(jobtoken,{host,accesstoken}={}) {
    var url = '/api/solve';
    if (host) url = host+url;

    let sendheaders = {};
    if (accesstoken !== undefined) headers["X-Mosek-Access-Token"] = accesstoken;

    let {status,data,headers} = await raw_GET(url,{query:{token:jobtoken},headers:sendheaders});
    let res = getHeaderValue("X-Mosek-Res-Code",headers);
    let trm = getHeaderValue("X-Mosek-Trm-Code",headers);
    let solfmt = getHeaderValue("ContentType",headers);
    if (! solfmt) solfmt = "application/binary";

    let msg = undefined;
    let sol = undefined;

    if (status == 200) {
        if (res == "MSK_RES_OK")
            sol = data;
        else
            msg = data;
        return {sol:sol,solfmt:solfmt,res:res,trm:trm,msg:msg};
    }
    else {
        return {res:res,trm:trm,msg:msg};
    }
}

// jobtoken : string
// onreply  : optional function(solution  : optional string,
//                              solformat : optional string,
//                              res       : optional string,
//                              trm       : optional string,
//                              msg       : optional string)
//    Where:
//       - solution is the solution file as a string.
//       - solformat is the MIME type if the solution
//       - res is the solver response code
//       - trm is the solver termination code
//       - msg is the error message of res!=MSK_RES_OK
//    If the solver fails completely, solution and solformat are
//    undefined, and res,trm and msg may or may not be defined,
//    otherwise solution and solformat are defined.
// data        : optional dictionary
//    The dictionary defines additional options
//    - "access-token" is an access token to use
//    - "accept" Requested MIME Type of result, "application/x-mosek-task",
//      "application/x-mosek-jtask" or "text/plain"
export async function solution(jobtoken,{host,accesstoken}={}) {
    var url = "/api/solution"
    if (host) url = host+url;

    let sendheaders = {};
    if (accesstoken !== undefined) sendheaders["X-Mosek-Access-Token"] = accesstoken;

    let {status,data,headers} = await raw_GET(url,{query:{token:jobtoken},headers:sendheaders});
    let res    = getHeaderValue("X-Mosek-Res-Code",headers);
    let trm    = getHeaderValue("X-Mosek-Trm-Code",headers);
    let solfmt = getHeaderValue("ContentType",headers);

    let msg = undefined;
    let sol = undefined;

    if (status == 200) {
        if (res == "MSK_RES_OK")
            sol = data;
        else
            msg = data;
        return {sol:sol,solfmt:solfmt,res:res,trm:trm,msg:msg};
    }
    else {
        return {res:res,trm:trm,msg:msg};
    }
}


// jobtoken : string
// onreply  : optional function(res       : optional string,
//                              trm       : optional string,
//                              msg       : optional string)
//    Where:
//       - res is the solver response code
//       - trm is the solver termination code
//       - msg is the error message of res!=MSK_RES_OK
//    Res,trm and msg may or may not be defined, depending on the
//    state of the solver.
// data        : optional dictionary
//    The dictionary defines additional options
//    - "access-token" is an access token to use
//      "application/x-mosek-jtask" or "text/plain"
export async function checkSolution(jobtoken,{host,accesstoken}={}) {
    var url = '/api/solution';
    if (host) url = host+url;

    let sendheaders = {};
    if (accesstoken !== undefined) sendheaders["X-Mosek-Access-Token"] = accesstoken;

    let {headers} = await raw_HEAD(url,{query:{token:jobtoken},headers:sendheaders});
    let res = getHeaderValue("X-Mosek-Res-Code",headers);
    let trm = getHeaderValue("X-Mosek-Trm-Code",headers);

    return {res:res,trm:trm,msg:msg};
}


// jobtoken : string
// onreply  : optional function(log : string)
// data        : optional dictionary
//    The dictionary defines additional options
//    - "access-token" is an access token to use
//    - "accept" Requested MIME Type of result, "application/x-mosek-task",
//      "application/x-mosek-jtask" or "text/plain"
//    - "offset" defines the offset to get log from
export async function log(jobtoken,{host,accesstoken,offset}={}) {
    var url = '/api/log';
    if (host) url = host+url;

    let headers = {};
    if (accesstoken !== undefined) headers["X-Mosek-Access-Token"] = accesstoken;
    let query = {token:jobtoken};
    if (offset !== undefined)
        query.offset = offset;

    let text = await GET(url,{query:query, headers:headers});
    return text === undefined ? "" : text;
}

// jobtoken : string
// onreply  : optional function(ok : bool)
// data        : optional dictionary
//    The dictionary defines additional options
//    - "access-token" is an access token to use
export async function stopJob(jobtoken,{host,accesstoken}={}) {
    var url = "/api/break";
    if (host) url = host+url;

    let headers = {};
    if (accesstoken !== undefined) headers["X-Mosek-Access-Token"] = accesstoken;

    return await GET(url,{query:{token:jobtoken}, headers:headers});
}

// jobtoken : string
// onreply  : optional function(ok : bool)
// data        : optional dictionary
//    The dictionary defines additional options
//    - "access-token" is an access token to use
export async function deleteJob(jobtoken,{host,accesstoken}={}) {
    var url = "/api/delete";

    if (host) url = host+url;

    let headers = {};
    if (accesstoken !== undefined) headers["X-Mosek-Access-Token"] = accesstoken;

    return await GET(url,{query:{token:jobtoken}, headers:headers});
}


////////////////////////////////////////////////////////////
//
// STATISTICS
//
////////////////////////////////////////////////////////////

export async function getStatistics({host} = {}) {
    let url = "/users/api/stats";
    if (host !== undefined) url = host+url;
    let data = await GET(url);
    return data === undefined ? undefined : JSON.parse(data);
}
