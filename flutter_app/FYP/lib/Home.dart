import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:fyp_app/widget/MapDialog.dart';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:flutter_joystick/flutter_joystick.dart';
import 'package:photo_view/photo_view.dart';
import 'package:web_socket_channel/status.dart' as status;
import 'dart:typed_data';
import 'dart:convert';
import 'dart:async';

class Home extends StatefulWidget {
  const Home({Key? key}) : super(key: key);

  @override
  State<Home> createState() => _HomeState();
}

final default_address = "10.164.38.36";

// final url = "ws://"+address+":5000";
// final control_url = "ws://"+address+":5000/control";
// final move_url = "ws://"+address+":5000/move";
// final track_result_url = "ws://"+address+":5000/track";

class _HomeState extends State<Home> {
  final StreamController streamController = StreamController.broadcast();

  WebSocketChannel? _socket;
  WebSocketChannel? _control_socket;
  WebSocketChannel? _move_socket;
  WebSocketChannel? _track_socket;
  int selected_index = -1;
  double ori_x = 0;
  bool _isConnected = false;
  bool _track = false;
  bool _auto_touring = false;

  String address = default_address;


  void connect(BuildContext context) async {
    String url = "ws://"+address+":5000";
    String control_url = "ws://"+address+":5000/control";
    String move_url = "ws://"+address+":5000/move";
    String track_result_url = "ws://"+address+":5000/track";

    _socket = WebSocketChannel.connect(Uri.parse(url));
    _control_socket = WebSocketChannel.connect(Uri.parse(control_url));
    _move_socket = WebSocketChannel.connect(Uri.parse(move_url));
    _track_socket = WebSocketChannel.connect(Uri.parse(track_result_url));

    streamController.addStream(_track_socket!.stream);

    _move_socket?.stream.listen((event) {
    });

    _control_socket?.stream.listen((event) {
    });

    setState(() {
      _isConnected = true;
    });

  }

  void reconnect(BuildContext context) async {
    String url = "ws://"+address+":5000";
    String control_url = "ws://"+address+":5000/control";
    String move_url = "ws://"+address+":5000/move";
    String track_result_url = "ws://"+address+":5000/track";

    _socket = WebSocketChannel.connect(Uri.parse(url));
    _control_socket = WebSocketChannel.connect(Uri.parse(control_url));
    _move_socket = WebSocketChannel.connect(Uri.parse(move_url));
    _track_socket = WebSocketChannel.connect(Uri.parse(track_result_url));

    streamController.addStream(_track_socket!.stream);

    _move_socket?.stream.listen((event) {
    });

    _control_socket?.stream.listen((event) {
    });

    setState(() {
      _isConnected = true;
    });
  }

  void disconnect() {
    _socket?.sink.close(status.goingAway);
    setState(() {
      _isConnected = false;
    });
  }

  void _on_pointer_down(PointerEvent details){
    ori_x = details.position.dx;
  }

  void _on_pointer_move(PointerEvent details){
    var dif_x = details.position.dx - ori_x;
  }

    @override
  Widget build(BuildContext context) {

    return DefaultTabController(
      initialIndex:0,
      length: 3,
      child: Scaffold(
          appBar: AppBar(
            title: Row(
              mainAxisAlignment: MainAxisAlignment.spaceBetween,
              children: [
                Text("Robot Controller"),
                ElevatedButton(
                    onPressed: () => reconnect(context),
                    child: Icon(
                      Icons.refresh
                    )
                ),
              ],
            )

          ),
          body: SafeArea(
            child: Column(
              crossAxisAlignment: CrossAxisAlignment.stretch,
              children: [

                !_isConnected? Padding(
                  padding: EdgeInsets.all(5),
                  child: TextFormField(
                    initialValue: address,
                    onChanged: (text) {
                      address = text;
                    },
                    decoration: const InputDecoration(
                      border: UnderlineInputBorder(),
                      labelText: 'Enter IP address',
                    ),
                  ),
                ): Container(),
                !_isConnected?Padding(
                  padding: const EdgeInsets.symmetric(vertical: 4, horizontal: 8.0),
                  child:ElevatedButton(
                    onPressed: () => connect(context),
                    child: const Text("Connect"),
                  ),
                ):Container(),
                _isConnected
                    ? Stack(
                      children: [
                        Container(
                          height: 270,
                          child: StreamBuilder(
                            stream: _socket?.stream,
                            builder: (context, snapshot) {
                          if (!snapshot.hasData) {
                            return Center(
                              child: Container(
                                  width: 25,
                                  height: 25,
                                child: const CircularProgressIndicator()
                              ),
                            );
                          }

                          if (snapshot.connectionState == ConnectionState.done) {
                            return const Center(
                              child: Text("Connection Closed !"),
                            );
                          }

                          //? Working for single frames
                          return Listener(
                            onPointerDown: _on_pointer_down,
                            onPointerMove: _on_pointer_move,
                            child: Image.memory(
                              Uint8List.fromList(
                                base64Decode(
                                  (snapshot.data.toString()),
                                ),
                              ),

                              gaplessPlayback: true,
                              excludeFromSemantics: true,
                            ),
                            );
                            },
                          ),
                        ),
                        Row(
                          mainAxisAlignment: MainAxisAlignment.end,
                          children: [
                            TextButton(
                                onPressed: () => showDialog<String>(
                                  context: context,
                                  builder: (BuildContext context) => Dialog(
                                    child: Padding(
                                      padding: const EdgeInsets.all(8.0),
                                      child: Column(
                                        mainAxisSize: MainAxisSize.min,
                                        mainAxisAlignment: MainAxisAlignment.center,
                                        children: <Widget>[
                                          PhotoView(
                                            imageProvider: AssetImage("assets/slam_map.PNG"),
                                            tightMode: true,
                                            disableGestures: true,
                                          ),
                                        ],
                                      ),
                                    ),
                                  ),
                                ),
                                style: ButtonStyle(
                                  backgroundColor: MaterialStateProperty.all<Color>(Color.fromRGBO(150,150,150,0.5)),
                                ),
                                child: Row(
                                  crossAxisAlignment: CrossAxisAlignment.center,
                                  children: [
                                    Icon(
                                      Icons.map,
                                      color: Colors.black,
                                    ),
                                    Text('Map',
                                      style: TextStyle(
                                          color: Colors.black
                                      ),
                                    ),
                                  ],
                                )
                            ),
                            SizedBox(
                              width: 10,
                            )
                          ],
                        ),
                      ],
                    ): Center(child: const Text("Initiate Connection")),
                _isConnected? Column(
                    children: [
                      TabBar(
                        isScrollable: true,
                        labelColor: Colors.black,
                        unselectedLabelColor: Colors.black12,
                        tabs: [
                          Tab(
                              child:Row(
                                mainAxisAlignment: MainAxisAlignment.center,
                                crossAxisAlignment: CrossAxisAlignment.center,
                                children: [
                                  Tab(
                                    child:Row(
                                      mainAxisAlignment: MainAxisAlignment.center,
                                      crossAxisAlignment: CrossAxisAlignment.center,
                                      children: [
                                        Icon(Icons.control_camera_sharp),
                                        SizedBox(
                                          width: 5,
                                        ),
                                        Text("Control")
                                      ],
                                    )
                                ),
                                ],
                              )
                          ),
                          Tab(
                              child:Row(
                                mainAxisAlignment: MainAxisAlignment.center,
                                crossAxisAlignment: CrossAxisAlignment.center,
                                children: [
                                  Icon(Icons.follow_the_signs),
                                  SizedBox(
                                    width: 5,
                                  ),
                                  Text("Tracking")
                                ],
                              )
                          ),
                          Tab(
                              child:Row(
                                mainAxisAlignment: MainAxisAlignment.center,
                                crossAxisAlignment: CrossAxisAlignment.center,
                                children: [
                                  Icon(Icons.auto_mode),
                                  SizedBox(
                                    width: 5,
                                  ),
                                  Text("AutoMode")
                                ],
                              )
                          ),
                        ],
                      ),
                    ],
                ):Container(),
                _isConnected?
                  Expanded(
                    child: TabBarView(
                      physics: const NeverScrollableScrollPhysics(),
                      children: [
                        Padding(
                          padding: EdgeInsets.symmetric(vertical: 10,horizontal: 10),
                          child:Joystick(
                            listener: (details){
                              var obj = {
                                "y": details.x*-1,
                                "x": details.y*-1,
                                "track":0,
                              };
                              _control_socket?.sink.add(jsonEncode(obj));
                            },

                            mode: JoystickMode.all,
                          ),
                        ),
                        Padding(
                          padding: EdgeInsets.symmetric(vertical:5,horizontal: 10),
                          child:Column(
                            crossAxisAlignment: CrossAxisAlignment.stretch,
                            children: [
                              ElevatedButton(
                                child:Text(_track?"Stop tracking":"Start to Track"),
                                onPressed: (){
                                  var obj = {
                                    "x": 0,
                                    "y": 0,
                                    "track":(_track)?-1:1
                                  };
                                  _track = (_track)?false:true;
                                  setState(() {});
                                  _control_socket?.sink.add(jsonEncode(obj));
                                },
                                style: ButtonStyle(
                                  backgroundColor:(_track)?MaterialStateProperty.all(Colors.red):MaterialStateProperty.all(Colors.blue)
                                ),
                              ),
                              Container(
                                child: _track?StreamBuilder(
                                  stream: streamController.stream,

                                  builder: (context, snapshot) {
                                    if (!snapshot.hasData) {
                                      return Center(
                                        child: Container(
                                          padding: EdgeInsets.symmetric(vertical: 10),
                                            child: Text("Nothing")
                                        ),
                                      );
                                    }

                                    Map<String, dynamic> map  = jsonDecode(snapshot.data);

                                    List<dynamic> detection_data = jsonDecode(map['data']);


                                    if (snapshot.connectionState == ConnectionState.done) {
                                      return const Center(
                                        child: Text("Detection closed"),
                                      );
                                    }

                                    return Expanded(
                                      child: SingleChildScrollView(
                                          child: Column(
                                            children: [
                                              Text("Detected Object",
                                                style: TextStyle(
                                                    fontSize: 20,
                                                    fontWeight: FontWeight.w500
                                                ),
                                              ),
                                              SizedBox(height: 10),
                                              Container(
                                                child: Column(
                                                  children: detection_data.map((obj) =>
                                                      Container(
                                                        padding: EdgeInsets.symmetric(vertical: 8),
                                                        child: InkWell(
                                                          onTap: (){
                                                            setState(() {
                                                              selected_index = obj["track_id"];
                                                            });
                                                            print(obj["track_id"]);
                                                          },
                                                          child: Column(
                                                            children: [
                                                              Row(
                                                                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                                                                crossAxisAlignment: CrossAxisAlignment.center,
                                                                children: [
                                                                  Row(
                                                                    crossAxisAlignment: CrossAxisAlignment.center,
                                                                    children: [
                                                                      Text(
                                                                        obj["track_id"].toString(),
                                                                        style: TextStyle(
                                                                            fontSize: 16,
                                                                            fontWeight: FontWeight.w300
                                                                        ),
                                                                      ),
                                                                      SizedBox(width: 10),
                                                                      Text(
                                                                        obj["name"].toString(),
                                                                        style: TextStyle(
                                                                            fontSize: 18,
                                                                            fontWeight: FontWeight.w500
                                                                        ),
                                                                      )
                                                                    ],
                                                                  ),
                                                                  Text(
                                                                    obj["confidence"].toStringAsFixed(2),
                                                                    style: TextStyle(
                                                                        fontSize: 16,
                                                                        fontWeight: FontWeight.w300
                                                                    ),
                                                                  ),
                                                                ],
                                                              ),
                                                              SizedBox(height: 5),
                                                              Divider(
                                                                color: (selected_index==obj["track_id"])?Colors.red:Colors.grey,
                                                                height: 1,
                                                                thickness: (selected_index==obj["track_id"])?2:0.5,
                                                              )
                                                            ],
                                                          ),
                                                        ),
                                                      )

                                                  ).toList()
                                                ),
                                              ),
                                              ElevatedButton(onPressed: (){
                                                var obj = {
                                                  "follow_obj_id":selected_index,
                                                };
                                                _control_socket?.sink.add(jsonEncode(obj));
                                              }, child: Text("Start to follow"))
                                            ],
                                          )
                                      )
                                    );
                                  },
                                ):Container(),
                              )
                            ],
                          )
                        ),
                        Padding(
                          padding: EdgeInsets.all(10),
                          child: Container(
                            child: Column(
                              crossAxisAlignment: CrossAxisAlignment.stretch,
                              children: [
                                !_auto_touring?ElevatedButton(onPressed: (){
                                  _auto_touring = !_auto_touring;
                                  setState(() {

                                  })
                                  ;
                                  var obj = {
                                    "auto_touring":_auto_touring,
                                  };
                                  _control_socket?.sink.add(jsonEncode(obj));
                                }, child: Text("Start auto touring")) :
                                ElevatedButton(
                                    style: ButtonStyle(
                                      backgroundColor: MaterialStateProperty.all(Colors.red)
                                    ),
                                    onPressed: (){
                                  _auto_touring = !_auto_touring;
                                  setState(() {

                                  })
                                  ;
                                  var obj = {
                                    "auto_touring":_auto_touring,
                                  };
                                  _control_socket?.sink.add(jsonEncode(obj));
                                }, child: Text("Cancel"))
                              ],
                            )
                          ),
                        )
                      ],
                    )
                ):Container()
              ],
            ),
          ),

      ),
    );
  }
}
