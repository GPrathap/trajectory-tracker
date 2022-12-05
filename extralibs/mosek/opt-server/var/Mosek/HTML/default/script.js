function splitext(fname) {
    var lastslash = fname.lastIndexOf('/');
    var lastdot   = fname.lastIndexOf('.');
    if (lastdot < lastslash) {
	return ["",""]
    } else {
	basename = fname.substring(0,lastdot);
	ext = fname.substring(lastdot,fname.length);
	return [basename,ext]
    }
}

function selectFile(event) {
    var files = event.target.files;
    if (files.length > 0) {
        var reader = new FileReader();
        reader.onload = function(f) {
            var req = new XMLHttpRequest();
            var data = f.target.result
            req.onreadystatechange = function() {
		if (req.readyState == 4) {
                    if (req.status = 200) {
			var token = req.responseText;
			// Send a solve request
			var req = new XMLHttpRequest();
			req.onreadystatechange = function() {
			    if (req.readyState == 4) {
				if req.status = 200 {
				    var parent = document.getElementById("row-result");
				    parent.innerHTML = "";
				    var node = document.createElement("pre");
				    var text = document.createTextNode(req.responseText);
				    node.appendChild(text);
				    parent.appendChild(node);
				}
			    }
			}
			req.open("GET", "/api/solve?"+token, true);
                    }
		}
		req.open("POST", "/api/submit", true);
		var basename_ext = splitext(f.name);
		var ext = basename_ext[1];
		var ext2 = ""
		
		if (ext == '.gz' || ext == '.zst' || ext == '.bz2' || ext == '.z') {
		    ext2 = ext
		    basename_ext = splitext(basename_ext[0]);
		    ext = basename_ext[1]
		}

		var contentTypeExt = ""
		var contentBaseType = "auto"
		if      ( ext2 == '.zst') contentTypeExt = "+zstd";
		else if ( ext2 == '.gz')  contentTypeExt = "+gzip";
		else if ( ext2 == '.bz2') contentTypeExt = "+bzip2";
		
		if ( ext == 'opf'  ) contentBaseType = "opf"
		if ( ext == 'lp'   ) contentBaseType = "lp"
		if ( ext == 'task' ) contentBaseType = "task"
		if ( ext == 'jtask') contentBaseType = "json"
		if ( ext == 'ptf'  ) contentBaseType = "ptf"
		if ( ext == 'mps'  ) contentBaseType = "mps"
		if ( ext == 'cbf'  ) contentBaseType = "cbf"

		req.setRequestHeader("ContentType","application/x-mosek-"+ext+ext2);
		req.send(data);
            }
        }
        reader.readAsBinaryString(f[0]);
    }
}
