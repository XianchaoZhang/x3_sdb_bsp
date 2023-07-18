document.write("<script type='text/javascript' src='js/websocket.js'></script>");
document.write("<script type='text/javascript' src='js/wfs/wfs.js'></script>");
document.write("<script type='text/javascript' src='js/jquery.min.js'></script>");

var g_chpi_type = "";
var g_app_version = "";
var g_sensor_list = 0;
var g_solution_configs = "";

var g_play_stream_chn = 0;


// 页面加载后立即请求连接 websocket 服务器
// 连接成功后，同步时间、获取能力集、调整UI、拉流
var serverIp = window.location.host;
$(document).ready(function()
{
    // alert("serverIp:" + serverIp);
    // 链接 websocket 服务器
    // 发起连接
    socket.init("ws://" + serverIp +":4567")
});


function ws_send_cmd_switch_app(data)
{
	var cmd = {
		kind: 1, //请求类型 kind 1 应用切换
		'param': data,
	}
	socket.send(cmd)
}


function ws_send_cmd_snap(data)
{
	var cmd = {
		kind: 2, //请求类型 kind 2 抓拍
		'snap_type': data,
	}
	socket.send(cmd)
}


function ws_send_cmd_start_stream(data)
{
	var cmd = {
		kind: 3, //请求类型 kind 3 websocket 开始推流
		'stream_chn': data,
	}
	socket.send(cmd)
}

function ws_send_cmd_stop_stream(data)
{
	var cmd = {
		kind: 4, //请求类型 kind 4 websocket 停止推流
		'stream_chn': data,
	}
  socket.send(cmd)
}

function ws_send_cmd_sync_time()
{
	var date = new Date();
	/*console.log("time: " + date.getTime() + "num time:" + Number(date.getTime()));*/
	var cmd = {
		kind: 5, //请求类型 kind 5 时间同步
		'time': Number(date.getTime()/1000),
	}
	socket.send(cmd);
}

function ws_send_cmd_get_hard_capability()
{
	var cmd = {
		kind: 6,
	}
	socket.send(cmd);
}

function ws_send_cmd_set_bitrate(data)
{
	var cmd = {
		kind: 7, //请求类型 kind 7 设置编码码率
		'bitrate': Number(data),
	}
	socket.send(cmd)	
}

function ws_send_cmd_save_configs(data)
{
	var cmd = {
		kind: 8,
		'param': data,
	}
	socket.send(cmd)	
}

function ws_send_cmd_recovery_configs()
{
	var cmd = {
		kind: 9,
	}
	socket.send(cmd)	
}

function draw_yolo5_result(alog_result) {
	//获取画布DOM  还不可以操作

	// 获取canvas的上层容器的大小，把 canvas 的大小设置得和他一样大
	var canvas_overlay = document.getElementById("canvas_overlay");

	var canvas = document.getElementById("canvas");    
	var context2D = canvas.getContext("2d"); 

	canvas.width = canvas_overlay.offsetWidth;
	canvas.height = canvas_overlay.offsetHeight;

	// 原图的大小是 1920 * 1080 或者 3840 * 2160
	// web上显示的大小不一定是这个值，所以画框的时候需要做比例调整
	// 获取真实视频的分辨率
	var video = document.getElementById("video");
	
	var h_ratio = canvas.height * 1.0 / video.videoHeight;
	var w_ratio = canvas.width * 1.0 / video.videoWidth;

	// 清空
	context2D.clearRect(0, 0, canvas.width, canvas.height);    
	context2D.globalAlpha = 50;

	// 画矩阵
	/*context2D.fillStyle = '#FF0000'; //颜色填充*/
    /*context2D.fillRect(50, 50, 30, 60);*/


	//将上面的圆填充为灰色  		
	/*context2D.fillStyle ="#FF0000"; */
	/*context2D.arc(30, 30, 15, 0, Math.PI*2 , false);    //注意这里的参数是弧度制，而不是角度制*/
	/*context2D.fill(); */

	// 画线
	/*context2D.moveTo(0,0);*/
	/*context2D.lineTo(100,100);*/
	/*context2D.stroke();*/

	// 画框
	/*context2D.lineWidth=1;*/
	/*context2D.strokeStyle="#f1af37";*/
	/*context2D.strokeRect(0, 0, 786 - 329, 989 - 408);*/
	/*context2D.stroke();*/

	// 写字
	/*context2D.font = "30px Arial";*/
	/*context2D.fillStyle = "#FF0000";	*/
    /*context2D.fillText("id", 200, 200);*/
	/*context2D.fill();*/

	// 遍历bbox
	context2D.lineWidth=1;
	context2D.strokeStyle="#f1af37";
	context2D.font = "24px Arial";
	context2D.fillStyle = "#FF0000";
	for (var i in alog_result) {
		/*console.log(alog_result[i]);*/
		var result = alog_result[i];
		context2D.strokeRect(result.bbox[0] * w_ratio, result.bbox[1] * h_ratio, (result.bbox[2] - result.bbox[0]) * w_ratio, (result.bbox[3] - result.bbox[1]) * h_ratio);
		context2D.fillText(result.name + "(" + result.score + ")", result.bbox[0] * w_ratio, result.bbox[1] * h_ratio);
	}
	context2D.stroke();
	context2D.fill();
}

function drawLine1(cxt, x, y, x1, y1, borderColor) {
	return;
	cxt.beginPath(); // 重新绘制

	cxt.lineWidth = 2;//线条的宽度
	cxt.strokeStyle = borderColor;//线条的颜色

	/*console.log(x, y, x1, y1);*/
    cxt.moveTo(x,y);
	cxt.lineTo(x1,y1);


	cxt.stroke(); // 绘制
}

// 在画布上连接关键点，起始坐标和结束坐标是两个关键点的中间
// ctx: 画布
// kps: 一组关键点
// ps、ps1: 与ps1共同计算得到中间位置，做为连线的起始点坐标
// pe、pe1: 与pe1共同计算得到中间位置，做为连线的结束点编号
// borderColor: 颜色
function drawLine(cxt, kps, ps, pe, ps1, pe1, borderColor, w_ratio, h_ratio) {
	// 过滤不可信的点
	if ( kps[ps*3 + 2] < 0.9 || kps[pe*3 + 2] < 0.9)
		return;
	if (ps1 != -1 && kps[ps1*3 + 2] < 0.9)
		return;
	if (pe1 != -1 && kps[pe1*3 + 2] < 0.9)
		return;

	var x=y=x1=y1=0;
	
	if (ps1 != -1) {
		x = (kps[ps*3 + 0] + kps[ps1*3 + 0]) * w_ratio / 2;
		y = (kps[ps*3 + 1] + kps[ps1*3 + 1]) * h_ratio / 2;
	} else {
		x = kps[ps*3 + 0] * w_ratio;
		y = kps[ps*3 + 1] * h_ratio;
	}

	if (pe1 != -1) {
		x1 = (kps[pe*3 + 0] + kps[pe1*3 + 0]) * w_ratio / 2;
		y1 = (kps[pe*3 + 1] + kps[pe1*3 + 1]) * h_ratio / 2;
	} else {
		x1 = kps[pe*3 + 0] * w_ratio;
		y1 = kps[pe*3 + 1] * h_ratio;
	}

	cxt.beginPath(); // 重新绘制

	cxt.lineWidth = 4;//线条的宽度
	cxt.strokeStyle = borderColor;//线条的颜色

	/*console.log(x, y, x1, y1);*/
    cxt.moveTo(x,y);
	cxt.lineTo(x1,y1);


	cxt.stroke(); // 绘制
}


function draw_person_result(alog_result)
{
	//获取画布DOM  还不可以操作

	// 获取canvas的上层容器的大小，把 canvas 的大小设置得和他一样大
	var canvas_overlay = document.getElementById("canvas_overlay");

	var canvas = document.getElementById("canvas");    
	var context2D = canvas.getContext("2d"); 

	canvas.width = canvas_overlay.offsetWidth;
	canvas.height = canvas_overlay.offsetHeight;

	// 原图的大小是 1920 * 1080 或者 3840 * 2160
	// web上显示的大小不一定是这个值，所以画框的时候需要做比例调整
	// 获取真实视频的分辨率
	var video = document.getElementById("video");
	
	var h_ratio = canvas.height * 1.0 / video.videoHeight;
	var w_ratio = canvas.width * 1.0 / video.videoWidth;

	// 清空
	context2D.clearRect(0, 0, canvas.width, canvas.height);    
	context2D.globalAlpha= 50;

	// 获取算法结果是否展示的属性框
	var checkboxs = document.getElementsByName('display_type');

	// 遍历bbox
	context2D.lineWidth=2;
	context2D.strokeStyle="#009688";
	context2D.font = "24px Arial";
	context2D.fillStyle = "#FF0000";
	if (alog_result.body && checkboxs[2].checked) {
		var bboxs = alog_result.body
		for (var i in bboxs) {
			/*console.log(bboxs[i]);*/
			var result = bboxs[i];
			if (result.score < 0.9)
				continue;
			context2D.strokeRect(result.bbox[0] * w_ratio, result.bbox[1] * h_ratio, (result.bbox[2] - result.bbox[0]) * w_ratio, (result.bbox[3] - result.bbox[1]) * h_ratio);
			/*context2D.fillText("(" + result.score + ")", result.bbox[0] * w_ratio, result.bbox[1] * h_ratio);*/
		}
	}
	context2D.strokeStyle="#1E9FFF";
	if (alog_result.face && checkboxs[0].checked) {
		var bboxs = alog_result.face
		for (var i in bboxs) {
			/*console.log(bboxs[i]);*/
			var result = bboxs[i];
			if (result.score < 0.9)
				continue;
			context2D.strokeRect(result.bbox[0] * w_ratio, result.bbox[1] * h_ratio, (result.bbox[2] - result.bbox[0]) * w_ratio, (result.bbox[3] - result.bbox[1]) * h_ratio);
			/*context2D.fillText("(" + result.score + ")", result.bbox[0] * w_ratio, result.bbox[1] * h_ratio);*/
		}
	}
	context2D.strokeStyle="#FF5722";
	if (alog_result.head && checkboxs[1].checked) {
		var bboxs = alog_result.head
		for (var i in bboxs) {
			/*console.log(bboxs[i]);*/
			var result = bboxs[i];
			if (result.score < 0.9)
				continue;
			context2D.strokeRect(result.bbox[0] * w_ratio, result.bbox[1] * h_ratio, (result.bbox[2] - result.bbox[0]) * w_ratio, (result.bbox[3] - result.bbox[1]) * h_ratio);
			/*context2D.fillText("(" + result.score + ")", result.bbox[0] * w_ratio, result.bbox[1] * h_ratio);*/
		}
	}
	context2D.stroke();
	context2D.fill();
	if (alog_result.kps && checkboxs[3].checked) {
		var kpss = alog_result.kps;
		for (var i in kpss) {
			var node = kpss[i];
			/*console.log(node);*/
			/* 算法结果编号，每个点包含坐标和分值
			
									2(左眼)	 1(右眼)

									
							     4(左耳)        	   3(右耳)
								
										 0(鼻子)



										 

					 6(左肩)										5(右肩)




					 8(左胳膊)										7(右胳膊)




				   10(左手腕)									    9(右手腕)


					          12(左跨)					11(右跨)









							  14(左膝盖)					13(右膝盖)











							  16(左脚踝)					15(右脚踝)
					         
			*/

			// 鼻子连接右眼
			drawLine(context2D, node.kps, 0, 1, -1, -1, "HotPink", w_ratio, h_ratio);
			// 鼻子连接左眼
			drawLine(context2D, node.kps, 0, 2, -1, -1, "SlateBlue", w_ratio, h_ratio);
			// 鼻子连接到右耳
			// drawLine(context2D, node.kps, 0, 3, -1, -1, "brown", w_ratio, h_ratio);
			// 鼻子连接到左耳
			// drawLine(context2D, node.kps, 0, 4, -1, -1, "orange", w_ratio, h_ratio);
			// 两眼连接
			// drawLine(context2D, node.kps, 1, 2, -1, -1, "white", w_ratio, h_ratio);
			// 右眼到右耳
			drawLine(context2D, node.kps, 1, 3, -1, -1, "MediumVioletRed", w_ratio, h_ratio);
			// 左眼到左耳
			drawLine(context2D, node.kps, 2, 4, -1, -1, "Purple", w_ratio, h_ratio);
			// 两耳连接
			// drawLine(context2D, node.kps, 3, 4, -1, -1, "springgreen", w_ratio, h_ratio);

			// 鼻子 - 左右肩中间
			drawLine(context2D, node.kps, 0, 5, -1, 6, "DodgerBlue", w_ratio, h_ratio);
			// 右肩 - 左右肩中间
			drawLine(context2D, node.kps, 5, 5, -1, 6, "DarkOrange", w_ratio, h_ratio);
			// 左肩 - 左右肩中间
			drawLine(context2D, node.kps, 6, 5, -1, 6, "Crimson", w_ratio, h_ratio);
		
			// 左右肩中间 - 左胯
			drawLine(context2D, node.kps, 5, 12, 6, -1, "LimeGreen", w_ratio, h_ratio);
			// 左右肩中间 - 右胯
			drawLine(context2D, node.kps, 5, 11, 6, -1, "DarkTurquoise", w_ratio, h_ratio);

			// 右肩 - 右胳膊
			drawLine(context2D, node.kps, 5, 7, -1, -1, "GreenYellow", w_ratio, h_ratio);
			// 右胳膊 - 右手
			drawLine(context2D, node.kps, 7, 9, -1, -1, "LawnGreen", w_ratio, h_ratio);
			// 左肩 - 左胳膊
			drawLine(context2D, node.kps, 6, 8, -1, -1, "Orange", w_ratio, h_ratio);
			// 左胳膊 - 左手
			drawLine(context2D, node.kps, 8, 10, -1, -1, "YellowGreen", w_ratio, h_ratio);

			// 右跨 - 右膝盖
			drawLine(context2D, node.kps, 11, 13, -1, -1, "CornflowerBlue", w_ratio, h_ratio);
			// 右膝盖 - 右脚踝
			drawLine(context2D, node.kps, 13, 15, -1, -1, "RoyalBlue", w_ratio, h_ratio);
			// 左跨 - 左膝盖
			drawLine(context2D, node.kps, 12, 14, -1, -1, "PaleGreen", w_ratio, h_ratio);
			// 左膝盖 - 左脚踝
			drawLine(context2D, node.kps, 14, 16, -1, -1, "Aquamarine", w_ratio, h_ratio);
}
	}

	
}

function show_mobilenetv2_result(msg) {
    var result = document.getElementById("mobilenetv2_result");
    result.innerHTML = msg;
	result.style.display = "block";
}

setInterval(() => {
	var canvas_overlay = document.getElementById("canvas_overlay");

	var canvas = document.getElementById("canvas");    
	var context2D = canvas.getContext("2d");
	context2D.clearRect(0, 0, canvas.width, canvas.height);    
	context2D.globalAlpha = 50;

	var result = document.getElementById("mobilenetv2_result");
    result.innerHTML = "";
	result.style.display = "none";
}, 1000); //1秒清除残留在画布上的算法渲染信息

setInterval(() => {
	/*console.log("show_play_frame");*/
	show_play_frame("播放帧率：" + socket.play_fps);
	socket.play_fps = 0;
	show_smart_fps("智能帧率：" + socket.smart_fps);
	socket.smart_fps = 0;
}, 1000); //1秒刷新一次帧率


function start_stream(chn) {
	var video = document.getElementById("video");
    wfs = new Wfs();
    wfs.attachMedia(video,'ch1');
	socket.wfs_handle = wfs;
	console.log("start stream chn: " + chn);
	
    ws_send_cmd_start_stream(Number(chn))

	g_play_stream_chn = Number(chn); // 更新当前播放流通道

	// 根据ip地址设置rtsp 拉流地址
	var text = document.getElementById("rtsp_url");
	text.innerHTML = "rtsp://" + window.location.host + "/stream_chn" + chn + ".h264";

	var ctips = document.getElementById("slt_sw_ctips"); 
    ctips.style.display = "none";

	var app_status = document.getElementById("app_status");
	app_status.style.display = "none";

	// 防止启动应用失败后导致界面显示不能根据x3M和x3e切换，所以只要打开流就获取一下设备信息
	//请求类型 kind 6 获取设备信息，如软件版本、芯片类型等
	ws_send_cmd_sync_time();
}

function stop_stream(chn) {
	console.log("stop stream chn: " + chn);

	// 退出拉流
	socket.wfs_handle.destroy();
	
	ws_send_cmd_stop_stream(Number(chn));
}

// 当视图页面不是激活状态时，退出拉流
// 当再次切回来时，重新拉流
document.addEventListener("visibilitychange", () => {
	console.log("visibilitychange: " + document.hidden);
    if (document.hidden) {
      	stop_stream(g_play_stream_chn);
    } else {     
        start_stream(g_play_stream_chn);
    }
});

function set_chip_type(chip_type)
{
	g_chpi_type = chip_type;
	if (chip_type === 'X3E') {
		var x3m_app = document.getElementById("x3m_4K");
		x3m_app.style.display = "none";   
	}

	var text = document.getElementById("chip_type");
	text.innerHTML = chip_type;
}

function set_app_version(app_version)
{
	g_app_version = app_version;
	var text = document.getElementById("app_version");
	text.innerHTML = app_version;
}

// bit0：F37
// bit1: IMX415
// bit2: OS8A10 OS8A10_2K
// bit3: OV8856
function set_sensor_list(sensor_list)
{
	g_sensor_list = sensor_list;
	// 根据sensor list 配置sensor list下来选项
	var sensor_name = document.getElementsByName("sensor_name");
	/*console.log(sensor_name);*/
	for (var i = 0; i < sensor_name.length; i++) {
		sel_sensor = sensor_name[i];
		var idx = 1;
		// 添加一个没有配置的选项，并且用来展示配置文件中的配置
		/*var option = new Option("", idx);*/
		/*sel_sensor.options[idx++] = option;*/
		if (1 & sensor_list) { // F37
			option = new Option("F37", idx);   
			sel_sensor.options[idx++] = option;
		}
		if (2 & sensor_list) {// IMX415
			option = new Option("IMX415", idx);   
			sel_sensor.options[idx++] = option;
		}
		if (4 & sensor_list) {// OS8A10
			option = new Option("OS8A10", idx);   
			sel_sensor.options[idx++] = option;
			option = new Option("OS8A10_2K", idx);   
			sel_sensor.options[idx++] = option;
		}
		if (8 & sensor_list) {// OV8856
			option = new Option("OV8856", idx);   
			sel_sensor.options[idx++] = option;
		}
		if (16 & sensor_list) {// SC031GS
			option = new Option("SC031GS", idx);   
			sel_sensor.options[idx++] = option;
		}
		if (32 & sensor_list) {// GC4663
			option = new Option("GC4663", idx);   
			sel_sensor.options[idx++] = option;
		}
		if (70 & sensor_list) {// IMX415_BV
			option = new Option("IMX415_BV", idx);   
			sel_sensor.options[idx++] = option;
		}
	}
}

function downloadFile(file_path) {   
    try{
		var pos = file_path.lastIndexOf('/');//'/所在的最后位置'
    	var file_name = file_path.substr(pos+1)//截取文件名称字符串
        var urlFile="http://" + serverIp + "/tmp_file/" + file_name;
		console.log('下载文件:' + urlFile)
        var elemIF = document.createElement("iframe");   
        elemIF.src = urlFile;   
        elemIF.style.display = "none";   
        document.body.appendChild(elemIF);   
    }catch(e){ 
		console.log('下载文件失败')
    } 
}

function show_app_status(msg) {
	var ctips = document.getElementById("slt_sw_ctips"); 
    ctips.style.display = "none";

	var app_status = document.getElementById("app_status");
	app_status.innerHTML = msg;
	app_status.style.display = "block";
}

// websocket每秒接收到多少视频帧
function show_play_frame(fps) {
	var frame_fps = document.getElementById("frame_fps");
	frame_fps.innerHTML = fps;
}

// webshocket 每秒接收到多少智能结果
function show_smart_fps(fps) {
	var frame_fps = document.getElementById("smart_fps");
	smart_fps.innerHTML = fps;
}


var is_dev_info_show = 0;
function show_dev_info() {
	var txt_dev_info = document.getElementById("dev_info");
	if (is_dev_info_show == 0)
		txt_dev_info.style.display = "block";
	else 
		txt_dev_info.style.display = "none";
	is_dev_info_show = is_dev_info_show ? 0 : 1;
}

var is_solution_configs_show = 0;
function show_solution_configs()
{
	var solution_configs = document.getElementById("solution_configs");
	if (is_solution_configs_show == 0)
		solution_configs.style.display = "table";
	else 
		solution_configs.style.display = "none";
	is_solution_configs_show = is_solution_configs_show ? 0 : 1;
}

var is_render_setting_show = 0;
function render_setting() {
	var obj = document.getElementById("render_setting");
	if (is_render_setting_show == 0)
		obj.style.display = "block";
	else 
		obj.style.display = "none";
	is_render_setting_show = is_render_setting_show ? 0 : 1;
}

// 显示方案框图
function btn_solution_id_click(i){
    var a = document.getElementById("solution_image"); 
    a.style.display = "block";

    if(i==1) {
        a.setAttribute("src","image/ipc-slt.jpg");
    }
    if(i==2) {
        a.setAttribute("src","image/usb-slt.jpg");
    }
    if(i==3) {
        a.setAttribute("src","image/box-slt.jpg");
    }
}


//监听选中事件
function view_stream_change() {
    //获取选中项的值
    var value = $("#selectStream option:selected").attr("value");
    //输出日志
    console.log("value=%s",value);
	// 关掉当前的拉流
    stop_stream(g_play_stream_chn);
    // 重新拉流
    g_play_stream_chn = Number(value);
    start_stream(g_play_stream_chn)
}

//监听选中事件
function view_bitrate_change() {
    //获取选中项的值
   var value = $("#selectBitRate option:selected").attr("value");
   if (value == "0") return;
   ws_send_cmd_set_bitrate(Number(value));
}

// 根据配置调整左侧控制区布局
function view_adjust_left_ui()
{
	// 恢复隐藏
	var div_snap_bnts = document.getElementById("div_snap_bnts");
	div_snap_bnts.style.display = "none";

	var div_sel_bitrate = document.getElementById("div_sel_bitrate");
	div_sel_bitrate.style.display = "none";

	var div_switch_stream = document.getElementById("div_switch_stream");
	div_switch_stream.style.display = "none";

	var div_personMulti_setting  = document.getElementById("div_personMulti_setting");
	div_personMulti_setting.style.display = "none";

	// 设置码率
	var selectBitRate = document.getElementById('selectBitRate');
	selectBitRate.options[0].selected = true;
	
	var solution_status = document.getElementById("solution_status");
	var status_txt = "";
	switch(g_solution_configs.solution_id) {
		case 0: // IPC
			status_txt += "当前场景： IP摄像机</br>";
			// 单目显示抓拍按钮
			if (g_solution_configs.ipc_solution.pipeline_num == 1) {
				status_txt += "单目应用</br>";
				status_txt += "Sensor: ";
				status_txt += g_solution_configs.ipc_solution.pipelines[0].sensor_name;
				status_txt += "</br>";

				div_snap_bnts.style.display = "block";
				div_sel_bitrate.style.display = "block";
				if (g_solution_configs.ipc_solution.pipelines[0].alog_id == 3)
					div_personMulti_setting.style.display = "block";

				selectBitRate.options[0].text = g_solution_configs.ipc_solution.pipelines[0].venc_bitrate + "kbps";
			}
			if (g_solution_configs.ipc_solution.pipeline_num == 2) {
				status_txt += "双目应用</br>";
				status_txt += "Sensor: ";
				status_txt += g_solution_configs.ipc_solution.pipelines[0].sensor_name;
				status_txt += " 和 ";
				status_txt += g_solution_configs.ipc_solution.pipelines[1].sensor_name;
				status_txt += "</br>";
				
				div_sel_bitrate.style.display = "block";
				div_switch_stream.style.display = "block";
			}
			break;
		case 1: // USB CAM
			status_txt += "当前场景： USB摄像机</br>";
			status_txt += "Sensor: ";
			status_txt += g_solution_configs.usb_cam_solution.pipeline.sensor_name;
			status_txt += "</br>";
			status_txt += "请连接USB接口</br>在PC或者android设备上预览视频</br>";
			status_txt += "</br>";
			status_txt += "<a href=\"/Download/com.shenyaocn.android.usbcamera.apk\" download=\"com.shenyaocn.android.usbcamera.apk\">下载uvc camera apk</a>";
			status_txt += "</br>";
			break;
		case 2: // VIDEO BOX
			status_txt += "当前场景： 智能盒子</br>";
			status_txt += "运行 ";
			status_txt += g_solution_configs.box_solution.box_chns;
			status_txt += " 路 1080P 编解码</br>";
			div_sel_bitrate.style.display = "block";
			selectBitRate.options[0].text = g_solution_configs.box_solution.venc_bitrate + "kbps";

			if (g_solution_configs.box_solution.alog_id == 3)
				div_personMulti_setting.style.display = "block";
			break;
		default:
			status_txt += "场景配置异常！"
			div_sel_bitrate.style.display = "block";
			if (g_solution_configs.box_solution.alog_id == 3)
				div_personMulti_setting.style.display = "block";
			break;
	}

	solution_status.innerHTML = status_txt;
}

function view_adjust_right_ui()
{
	// 调整配置页面
	// 根据配置设置下拉选项框的配置
	var venc_bitrates = document.getElementsByName("venc_bitrate");
	venc_bitrates[0].options[0].text = g_solution_configs.ipc_solution.pipelines[0].venc_bitrate + "kbps"; //ipc单目
	venc_bitrates[0].options[0].value = g_solution_configs.ipc_solution.pipelines[0].venc_bitrate;
	venc_bitrates[1].options[0].text = g_solution_configs.ipc_solution.pipelines[1].venc_bitrate + "kbps";; //ipc双目
	venc_bitrates[1].options[0].value = g_solution_configs.ipc_solution.pipelines[0].venc_bitrate;
	venc_bitrates[2].options[0].text = g_solution_configs.box_solution.venc_bitrate + "kbps";; //vidoe box
	venc_bitrates[2].options[0].value = g_solution_configs.box_solution.venc_bitrate;

	// 各场景的算法配置
	var alog_ids = document.getElementsByName("alog_id");
	var idx = g_solution_configs.ipc_solution.pipelines[0].alog_id;
	alog_ids[0].options[idx].selected = true;
	idx = g_solution_configs.ipc_solution.pipelines[1].alog_id;
	alog_ids[1].options[idx].selected = true;
	idx = g_solution_configs.usb_cam_solution.pipeline.alog_id;
	alog_ids[2].options[idx].selected = true;
	idx = g_solution_configs.box_solution.alog_id;
	alog_ids[3].options[idx].selected = true;

	// ipc和usb场景下的sensor配置
	var sensor_names = document.getElementsByName("sensor_name");
	sensor_names[0].options[0].text = g_solution_configs.ipc_solution.pipelines[0].sensor_name; //ipc单目
	sensor_names[1].options[0].text = g_solution_configs.ipc_solution.pipelines[1].sensor_name;
	sensor_names[2].options[0].text = g_solution_configs.usb_cam_solution.pipeline.sensor_name;

	// box场景下的路数配置
	var box_chns = document.getElementsByName("box_chns");
	if (g_solution_configs.box_solution.box_chns == 1)
		box_chns[0].options[0].selected = true;
	else if (g_solution_configs.box_solution.box_chns == 4)
		box_chns[0].options[1].selected = true;

	var solution_id = document.getElementsByName("solution_id");
	switch(g_solution_configs.solution_id) {
		case 0: // IPC
			solution_id[0].checked=true;
			var pipeline_num = document.getElementsByName("pipeline_num");
			if (g_solution_configs.ipc_solution.pipeline_num == 1) {
				pipeline_num[0].checked=true;
			}
			else if (g_solution_configs.ipc_solution.pipeline_num == 2) {
				pipeline_num[1].checked=true;				
			}
			break;
		case 1: // USB CAM
			solution_id[1].checked=true;
			break;
		case 2: // VIDEO BOX
			solution_id[2].checked=true;
			break;
		default:
			console.log("场景配置异常！");
			break;		
	}
}

// 根据sensor list和solution配置调整web ui选项
function view_adjust_ui()
{	
	view_adjust_left_ui();
	view_adjust_right_ui();
}

// 根据配置页面上的调整把配置项同步进json配置中
function sync_configs()
{
	var solution_id = document.getElementsByName("solution_id");
	for (var i = 0; i < solution_id.length; i++) {
		if (solution_id[i].checked)
			g_solution_configs.solution_id = i;
	}

	var pipeline_num = document.getElementsByName("pipeline_num");
	for (var i = 0; i < pipeline_num.length; i++) {
		if (pipeline_num[i].checked)
			g_solution_configs.ipc_solution.pipeline_num = i + 1;
	}

	var sensor_names = document.getElementsByName("sensor_name");
	var idx = sensor_names[0].selectedIndex;
	g_solution_configs.ipc_solution.pipelines[0].sensor_name = sensor_names[0].options[idx].text; //ipc单目
	var idx = sensor_names[1].selectedIndex;
	g_solution_configs.ipc_solution.pipelines[1].sensor_name = sensor_names[1].options[idx].text;
	var idx = sensor_names[2].selectedIndex;
	g_solution_configs.usb_cam_solution.pipeline.sensor_name = sensor_names[2].options[idx].text;

	var venc_bitrates = document.getElementsByName("venc_bitrate");
	var idx = venc_bitrates[0].selectedIndex;
	g_solution_configs.ipc_solution.pipelines[0].venc_bitrate = Number(venc_bitrates[0].options[idx].value);
	var idx = venc_bitrates[1].selectedIndex;
	g_solution_configs.ipc_solution.pipelines[1].venc_bitrate = Number(venc_bitrates[1].options[idx].value);
	var idx = venc_bitrates[2].selectedIndex;
	g_solution_configs.box_solution.venc_bitrate = Number(venc_bitrates[2].options[idx].value);

	// 各场景的算法配置
	var alog_ids = document.getElementsByName("alog_id");
	g_solution_configs.ipc_solution.pipelines[0].alog_id = alog_ids[0].selectedIndex;
	g_solution_configs.ipc_solution.pipelines[1].alog_id = alog_ids[1].selectedIndex;
	g_solution_configs.usb_cam_solution.pipeline.alog_id = alog_ids[2].selectedIndex;
	g_solution_configs.box_solution.alog_id = alog_ids[3].selectedIndex;

	var box_chns = document.getElementsByName("box_chns");
	if (box_chns[0].options[0].selected)
		g_solution_configs.box_solution.box_chns = 1;
	else if (box_chns[0].options[1].selected)
		g_solution_configs.box_solution.box_chns = 4;
	
	view_adjust_left_ui();
}

function switch_solution() {
	stop_stream(g_play_stream_chn);

	sync_configs();

	var ctips = document.getElementById("slt_sw_ctips"); 
    ctips.style.display = "block";

	ws_send_cmd_switch_app(JSON.stringify(g_solution_configs))
}

function save_solution_configs()
{
	sync_configs();	

	ws_send_cmd_save_configs(JSON.stringify(g_solution_configs));
}

function recovery_solution_configs()
{
	ws_send_cmd_recovery_configs();
}


function get_raw_frame() {
	ws_send_cmd_snap('raw')
}

function get_yuv_frame() {
    ws_send_cmd_snap('yuv')
}

function open_video() {
	if (navigator.mediaDevices === undefined) {
        navigator.mediaDevices = {};
    }
    //
    if (navigator.mediaDevices.getUserMedia === undefined) {
        navigator.mediaDevices.getUserMedia = function(constraints) {
            var getUserMedia = navigator.webkitGetUserMedia || navigator.mozGetUserMedia;
            if (!getUserMedia) {
                return Promise.reject(new Error('getUserMedia is not implemented in this browser'));
            }
            return new Promise(function(resolve, reject) {
                getUserMedia.call(navigator, constraints, resolve, reject);
            });
        }
    }

    window.URL = (window.URL || window.webkitURL || window.mozURL || window.msURL);
    var mediaOpts = {
        audio: false,
        video: true,
    }
    function successFunc(stream) {
        var video = document.getElementById("usb_video");
        if ("srcObject" in video) {
            video.srcObject = stream
        } else {
            video.src = window.URL && window.URL.createObjectURL(stream) || stream
        }
        video.play();
    }
    function errorFunc(err) {
        alert(err.name);
    }

    navigator.mediaDevices.getUserMedia(mediaOpts, successFunc, errorFunc);
}
function close_video() {
	var video = document.getElementById("usb_video");

    if (!video) return;
    let stream = video.srcObject
    console.log(stream);
    let tracks = stream.getTracks()
    tracks.forEach(track => {
        track.stop()
    })
    video.srcObject = null;
}



// websocket 连接成功
function ws_onopen()
{
	// 保证首次拉流时websocket已经连接
	if (Wfs.isSupported()) {   
		start_stream(0);
	}
	// sdb 板子上没有rtc保存时间，所以把pc的时间同步到设备上，让osd时间和pc时间同步
	ws_send_cmd_sync_time();


	// 请求类型 kind 6 获取设备信息，如软件版本、芯片类型等
	// 之后接收到设备能力信息后，根据能力集信息进行UI显示调整
	ws_send_cmd_get_hard_capability();
}

function handle_ws_recv(params) {
	/*console.log(params);*/
	if (params.kind == 1 && params.Status == 200) {
		start_stream(g_play_stream_chn);
    } else if (params.kind == 1 && params.app_status) {
		show_app_status(params.app_status);
	} else if (params.kind == 2) {
		downloadFile(params.Filename);
	} else if (params.kind == 3 && params.pipeline == g_play_stream_chn) {
		/*console.log(params);*/
		if (params.mobilenetv2_result) {
			/*console.log(params.mobilenetv2_result);*/
			show_mobilenetv2_result(params.mobilenetv2_result);
			socket.smart_fps++;
		}
		if (params.yolov5_result) {
			/*console.log(params.yolo5_result);*/
			draw_yolo5_result(params.yolov5_result);
			socket.smart_fps++;
		}
		if (params.fcos_result) {
			/*console.log(params.fcos_result);*/
			draw_yolo5_result(params.fcos_result);
			socket.smart_fps++;
		}
		if (params.person_result) {
			socket.smart_fps++;
			/*console.log(params.person_result);*/
			setTimeout(function(){draw_person_result(params.person_result)}, "600"); // 延时执行
		}
    } else if (params.kind == 6) {
		if (params.chip_type)
			set_chip_type(params.chip_type);
		if (params.app_version)
			set_app_version(params.app_version);
		if (params.sensor_list)
			set_sensor_list(params.sensor_list);
		if (params.solution_configs) {
			g_solution_configs = params.solution_configs;
			
			console.log(g_solution_configs);
			view_adjust_ui();
		}
    }
}
