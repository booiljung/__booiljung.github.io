# ScopeModel로 상태 관리 하기

2019년 8월 28일

Flutter는 여러가지 상태관리를 제공 합니다. 간단한 모델 관리 중 하나인 `ScopeModel`을 다루어 보겠습니다.







```
import 'package:flutter/material.dart';
import 'package:scoped_model/scoped_model.dart';


class CartModel extends Model {
  int count = 0;

  void increse() {
    count++;
    notifyListeners();    
  }

  static CartModel of(BuildContext context) =>
      ScopedModel.of<CartModel>(context);
}


void main() {
  final cart = CartModel();

  runApp(
    ScopedModel<CartModel>(
      model: cart,
      child: MyApp(),
    ),
  ); 
}



class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Flutter Demo',
      theme: ThemeData(
        primarySwatch: Colors.blue,
      ),
      home: MyHomePage(title: 'Flutter Demo Home Page'),
    );
  }
}



class MyHomePage extends StatefulWidget {
  MyHomePage({Key key, this.title}) : super(key: key);

  final String title;

  @override
  _MyHomePageState createState() => _MyHomePageState();
}



class _MyHomePageState extends State<MyHomePage> {

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.title),
      ),
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: <Widget>[
            Text(
              'You have pushed the button this many times:',
            ),
            ScopedModelDescendant<CartModel>(
              builder: (context, child, cart) {
                return Text(
                  '${CartModel.of(context).count}',
                  style: Theme.of(context).textTheme.display1,
                );
              },
            ),
          ],
        ),
      ),

      floatingActionButton: FloatingActionButton(
        onPressed: () => CartModel.of(context).increse(),
        tooltip: 'Increment',
        child: Icon(Icons.add),
      ),
    );
  }
}

```

