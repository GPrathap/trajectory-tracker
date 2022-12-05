
export async function request(url,method,{data,query,timeout,headers} = {}) {
    return await new Promise((resolve,reject) => {
	let req = new XMLHttpRequest();
	req.onreadystatechange = () => {
            if (req.readyState == 4) {
		switch (req.status) {
		case 200:
		case 204:
		    resolve({status:req.status,data:req.responseText,headers:req.getAllResponseHeaders()});
		    break;
		default:
		    reject("Status code = "+req.status);
		    break;
		}
	    }
	}

	req.onerror   = reject;
	req.onabort   = reject;
	req.ontimeout = reject;

	if (timeout !== undefined) { req.timeout = timeout; }

	let sendurl = url;
	if (query !== undefined) {
	    let q = [];
	    for (let k in query) { q.push(encodeURI(`${k}=${query[k]}`)); }
            if (q.length > 0)
	        sendurl += "?" + q.join('&');
	}

	req.open(method,sendurl,true);
        if (headers !== undefined) {
            for (let k in headers) {
                req.setRequestHeader(k,headers[k]);
            }
        }

        if (method == 'PUT' ||
            method == 'POST')
	    req.send(data);
        else
            req.send();
    });
}

// export async function raw_POST(url,data,{query,timeout,headers} = {}) {
//     return await new Promise((resolve,reject) => {
// 	let req = new XMLHttpRequest();
// 	req.onreadystatechange = () => {
//             if (req.readyState == 4) {
// 		switch (req.status) {
// 		case 200:
// 		    resolve(req);
// 		    break;
// 		case 204:
// 		    resolve(req);
// 		    break;
// 		default:
// 		    reject("Status code = "+req.status);
// 		    break;
// 		}
// 	    }
// 	}

// 	req.onerror   = reject;
// 	req.onabort   = reject;
// 	req.ontimeout = reject;

// 	if (timeout !== undefined) {
// 	    req.timeout = timeout;
// 	}

// 	let sendurl = url;
// 	if (query !== undefined) {
// 	    let q = [];
// 	    for (let k in query) { q.push(encodeURI(`${k}=${query[k]}`)); }
//             if (q.length > 0)
// 	        sendurl += "?" + q.join('&');
// 	}

// 	req.open('POST',sendurl,true);
//         if (headers !== undefined) {
//             for (let k in headers) {
//                 req.setRequestHeader(k,headers[k]);
//             }
//         }
// 	req.send(data);
//     });
// }

// export async function POST(url,data,kwds={}) {
//     let req = await raw_POST(url,data,kwds);
//     return  (req.status == 200 ?
//              req.responseText : undefined);
// }

// export async function raw_GET(url,{query,timeout,headers} = {}) {
//     return await new Promise((resolve,reject) => {
// 	let req = new XMLHttpRequest();
// 	req.onreadystatechange = function() {

// 	    if (req.readyState == 4) {
// 		switch (req.status) {
// 		case 200:
// 		case 204:
// 		    resolve(req);
// 		    break;
// 		default:
// 		    reject("Status code = "+req.status);
// 		    break;
// 		}
// 	    }
// 	}
// 	req.onerror   = reject;
// 	req.onabort   = reject;
// 	req.ontimeout = reject;

// 	if (timeout !== undefined) {
// 	    req.timeout = timeout;
// 	}


//         let sendurl = url;
// 	if (query !== undefined) {
// 	    let q = [];
// 	    for (let k in query) { q.push(encodeURI(`${k}=${query[k]}`)); }
//             if (q.length > 0)
// 	        sendurl += "?" + q.join('&');
// 	}

// 	req.open('GET',sendurl,true);
//         if (headers !== undefined) {
//             for (let k in headers) {
//                 req.setRequestHeader(k,headers[k]);
//             }
//         }
// 	req.send();
//     });
// }

// export async function raw_HEAD(url,{query,timeout,headers} = {}) {
//     return await new Promise((resolve,reject) => {
// 	let req = new XMLHttpRequest();
// 	req.onreadystatechange = () => {
// 	    if (req.readyState == 4) {
// 		switch (req.status) {
// 		case 200:
// 		case 204:
// 		    resolve(req);
// 		    break;
// 		default:
// 		    reject("Status code = "+req.status);
// 		    break;
// 		}
// 	    }
// 	}
// 	req.onerror   = reject;
// 	req.onabort   = reject;
// 	req.ontimeout = reject;

// 	if (timeout !== undefined) {
// 	    req.timeout = timeout;
// 	}


//         let sendurl = url;
// 	if (query !== undefined) {
// 	    let q = [];
// 	    for (let k in query) { q.push(encodeURI(`${k}=${query[k]}`)); }
//             if (q.length > 0)
// 	        sendurl += "?" + q.join('&');
// 	}

// 	req.open('HEAD',sendurl,true);
//         if (headers !== undefined) {
//             for (let k in headers) {
//                 req.setRequestHeader(k,headers[k]);
//             }
//         }
// 	req.send();
//     });
// }

// export async function GET(url,kwds={}) {
//     let req = await raw_GET(url,kwds);
//     return  (req.status == 200 ?
//              req.responseText : undefined);
// }
