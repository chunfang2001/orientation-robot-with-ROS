import 'package:flutter/material.dart';

class MapDialog extends StatelessWidget {
  const MapDialog({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Dialog(
      child: Container(
        width: 400,
        height: 400,
        decoration: BoxDecoration(
            image: DecorationImage(
                image: ExactAssetImage("assets/slam_map.PNG"),
                fit: BoxFit.cover
            )
        ),
      ),
    );
  }
}
