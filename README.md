# CoppeliaSim pan-tilt-camera simulation

### B0-based remote API

B0-based remote APIでclient-server通信を行うためには，以下を満たす環境を用意する．

PythonのMessagePackモジュールをインストール (pip)

```
$ pip3 install msgpack
```

CoppeliaSimのアプリケーションフォルダ(`/Applications/CoppeliaSim_Edu_V4_0_0_Mac`)に以下のフォルダが存在する．

- `programming/remoteApiBindings/b0Based/python/python/b0RemoteApi.py`
- `programming/remoteApiBindings/b0Based/python/python/b0.py`

blueZero (e.g. b0.dll) と依存するパッケージ (e.g. libzmq, boost_chrono, boost_system, boost_thread, etc.) が存在する．

!! blueZero について詳しく知りたい場合，以下を参照する

- https://bluezero.readthedocs.io/en/stable/install_bluez.html
- https://github.com/CoppeliaRobotics/bluezero

### How to Install blueZero (in MacOS)

blueZero (b0) に必要なパッケージのインストール (Homebrew)

```
$ brew install zeromq
$ brew install protobuf
$ brew install boost
$ brew install lz4
$ brew install qt

$ brew install doxygen
$ brew install graphviz
$ brew install mscgen
```

##### blueZero (b0) のインストール

~/.bashrcに以下を追加する．

```
# ~/.bashrcに以下を記載して反映させる
# QTのversionは任意
export QT5DIR=/usr/local/Cellar/qt/5.15.0
export CMAKE_MODULE_PATH=${QT5DIR}/lib/cmake:${CMAKE_MODULE_PATH}
export CMAKE_PREFIX_PATH=${QT5DIR}
```

`~/.bashrc`の変更内容を反映させる．

```
$ source ~/.bashrc

$ git clone --branch v1 --recursive https://github.com/BlueWorkforce/bluezero
$ cd bluezero
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_GUI=ON -DBUILD_EXAMPLES=ON ..
$ make
```

### B0-based remote APIでclient-server通信


##### Server-side

Model:「B0 Remote API Server」をSceneにドラッグ&ドロップする．
Scene直下に「b0RemoteApiServer.lua」というスクリプトが作成される．「b0RemoteApiServer.lua」には以下の記述があり，APIを読み込んでいる．

```
require('b0RemoteApiServer')
```

クライアント側から制御したいオブジェクトのchild script(.lua)の1行目に以下を追加する．クライアント側(シミュレータ)は，ポート番号:199999でサーバからの命令受信を行う．)

```
simRemoteApi.start(19999)
```

クライアント側と通信したいSceneで，シミュレーションを開始する．

##### Client-side

環境構築が終わったら，サンプルコード (simpleTest.py) を動かす．

```
$ cd /Applications/CoppeliaSim_Edu_V4_0_0_Mac/
$ cd programming/remoteApiBindings/python/python
$ cp ../../lib/lib/MacOS/remoteApi.dylib .
$ ls -l
...
sim.py
simConst.py
remoteApi.dylib # DLLファイル(APIの本体)
...
```

```
$ python3 simpleTest.py # 通信テストを実行
```

### Pantilt Camera simulation

##### Client-side

CoppeliaSimのシーンファイル(.ttt)内でシミュレーションを開始する．

##### Server-side

```
$ python3 main.py # パンチルトカメラをPythonから制御する．
```

`main.py`が扱う処理:

- CoppeliaSim(クライアント)内に配置されたパンチルトカメラ(Pan-Tilt Camera, PTZ Camera)の，ズーム，ヨー，ピッチの3つのパラメータをPythonスクリプト(サーバ)上で制御してみる． (サーバ → クライアント)
- CoppeliaSim(クライアント)内に配置されたパンチルトカメラ(Pan-Tilt Camera, PTZ Camera)の，視覚センサ(Vision sensor)が受け取った画像をPythonスクリプト(サーバ)上で制御してみる． (クライアント → サーバ)

