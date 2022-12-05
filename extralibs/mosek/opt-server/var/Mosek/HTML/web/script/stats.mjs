//-*-javascript-*-
import * as optapi from './optapi.mjs';

function drawStatWindow(node,timestamps,data,{yaxisunit,xaxisunit,ymax,ymul}={}) {
    let parent = node.parentNode;

    let width  = parent.clientWidth-2;
    let height = 200;

    node.style.width  = width+'px';
    node.style.height = height+'px';


    let min = data[0];
    let max = data[0];

    let num = data.length;
    for (let i = 1; i < num; ++i) {
        if (data[i] < min) min = data[i];
        if (data[i] > max) max = data[i];
    }
    if (ymul === undefined) ymul = 1.0;
    let T0i = 0; while (T0i < num && timestamps[T0i] == 0) ++T0i;
    let tmin = T0i < num ? timestamps[T0i] : 0;
    if (max <= min+1) max = min+1.01;
    max = Math.pow(10,Math.ceil(Math.log10(max)));
    if (ymax !== undefined) max = ymax*ymul;

    let xbase  = 100;
    let ybase  = 40;
    let yspan  = max>min ? max-min : height-ybase;
    let yscale = (height-ybase) / yspan;
    let xspan  = tmin == 0 ? 1 : 1-tmin;
    if      (tmin < 10*60*60)     tmin = 10*60*60;
    else if (tmin < 100*60*60)    tmin = 100*60*60;
    else if (tmin < 10*24*60*60)  tmin = 10*24*60*60;
    else if (tmin < 100*24*60*60) tmin = 100*24*60*60;

    //console.log("ymax=",max,", yspan=",yspan,",yscale=",yscale);


    let xscale = (width-xbase) / xspan;

    let ctx = node.getContext("2d");
    ctx.canvas.width  = width;
    ctx.canvas.height = height;

    ctx.fillStyle = "#ffffffff";
    ctx.fillRect(0,0,width,height);
    ctx.fillStyle = "#e0e0ffff";
    ctx.fillRect(xbase,0,width,height-ybase);

    ctx.fillStyle = "#000000ff";
    switch (yaxisunit) {
    case "%":
	ctx.textBaseline = "hanging";
	ctx.textAlign = "right";
	ctx.fillText(max+"%",xbase-5,0);
	ctx.textBaseline = "alphabetic";
	ctx.fillText("0%",  xbase-5,height-ybase);
	break;
    case "time":
	ctx.textBaseline = "hanging";
	ctx.textAlign = "right";
	ctx.fillText(max+" sec",xbase-5,0);
	ctx.textBaseline = "alphabetic";
	ctx.fillText(min+" sec",  xbase-5,height-ybase);
	break;
    default:
	ctx.textAlign = "right";
	ctx.textBaseline = "hanging";
	ctx.fillText(""+max,xbase-5,0);
	ctx.textBaseline = "alphabetic";
	ctx.fillText(""+min,xbase-5,height-ybase);
	break;
    }

    ctx.textAlign    = "right";
    ctx.textBaseline = "hanging";
    ctx.fillText("now",width,height-ybase);

    //console.log(xbase,ybase);

    // ctx.moveTo(width-timestamps[0]*xscale,height-data[0]*yscale-ybase);
    // for (let i = 1; i < num; ++i) {
    //     ctx.lineTo(width-timestamps[i]*xscale,height-data[i]*yscale-ybase);
    // }
    // ctx.stroke();
    {
        // console.log("data = ",data.slice(T0i,data.length));
        //console.log("  ymul =",ymul,", yscale = ",yscale);
        for (let i = T0i; i < num; ++i) {
            // console.log(width+(timestamps[i]-1)*xscale,
            //             height-ybase-data[i]*ymul*yscale-1,
	    //      	Math.ceil(width/(num-T0i)),
            //             data[i]*yscale*ymul+1);
            ctx.fillRect(width+(timestamps[i]-1)*xscale,
                         height-ybase-data[i]*ymul*yscale-1,
			 Math.ceil(width/(num-T0i)),
                         data[i]*yscale*ymul+1);
        }
    }
}

function updateStatViews(data) {
    {
        let node = document.getElementById("stat-cpu-graph");
        node.data = data.Cpu;
        drawStatWindow(node,data.Timestamps,data.Cpu,{ yaxisunit:"%",ymul:100,ymax:1 });
    }
    {
        let node = document.getElementById("stat-avg-queue-time-graph");
        node.data = data.JobQueueTime;
        drawStatWindow(node,data.Timestamps,data.JobQueueTime,{ yaxisunit:"time" });
    }
    {
        let node = document.getElementById("stat-avg-run-time-graph");
        node.data = data.JobAvgTime;
        drawStatWindow(node,data.Timestamps,data.JobAvgTime,{ yaxisunit:"time" });
    }
    if (true) {
    {
        let node = document.getElementById("stat-jobs-added-graph");
        node.data = data.JobsAdded;
        drawStatWindow(node,data.Timestamps,data.JobsAdded);
    }
    {
        let node = document.getElementById("stat-jobs-enqueued-graph");
        node.data = data.JobsStarted;
        drawStatWindow(node,data.Timestamps,data.JobsStarted);
    }
    {
        let node = document.getElementById("stat-jobs-completed-graph");
        node.data = data.JobsCompleted;
        drawStatWindow(node,data.Timestamps,data.JobsCompleted);
    }
    }
}


async function startViewUpdater() {
    let data = await optapi.getStatistics();
    updateStatViews(data);

    {
        let node = document.getElementById("data-num-cpus");
        node.innerHTML = ""+data.NCpus;
    }
    {
        let node = document.getElementById("data-sample-interval");
        node.innerHTML = ""+data.SampleInterval;
    }

    let interval = data.SampleInterval < 1 ? 1 : data.SampleInterval;
    window.setInterval(
        async () => {
            let data = await optapi.getStatistics();
            updateStatViews(data);
        },
        interval*1000);
}



export function initialize() {
    startViewUpdater()
}
