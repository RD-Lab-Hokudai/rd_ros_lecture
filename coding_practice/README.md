coding\_practice
====

# ROSのnodeのC++でのコーディング例

## Topicをpublish/subscribeする例
### `pub_and_sub_node`
* `while` loopによる周期的なpublish
* subscribe時のcallbackでpublishする
* std\_msgs/String `chatter`とvisualization\_msgs/Marker 'doodle' をpublish
* std\_msgs/Int32 `number`をsubscribe

## Service serverである例
### `services_server_node`
* 複数のserviceのserverとなっているnode

## Service clientである例
### `services_client_node`
* serviceを呼ぶclientとなっているnode

## Service ServerかつService Clientである例
### `services_server_client_node`
* 外部のserviceと自身のserviceをcallするnode

## Topicをsubscribeし，かつService serverである例
### `subscriber_and_service_server_node`
* `spin`によりTopicをService clientを待ち受けるnode

## coding\_practice pkg内に独自のmsg/srvを作り利用する例
### `coding\_practice/practice\_data.msg`
* TBD

### `coding\_practice/practice\_func.srv`
* TBD

## 他のpkgであるbasic\_lectureのmsg/srv型を利用する例
### `selfbuild_msg_srv_node`
* catkinワークスペース内に自作したmsg/srvを扱うnode

## Topicを周期的にpublish，別のTopicをsubcribeして，かつService server/clientである例
### `multifunction_node`
* TBD

## `while()`を使わないで周期的なpublishをする例
### `timer_publisher_node`
* `while()`ではなくtimerによるCallbackで周期的な処理をする

## node内でsubscribeしているTopicをpublishしているnodeの数，またその逆について調べる例
### `connection_checker_node`
* TBD

## nodeでなくてライブラリを作る例
### `libcoding_practice`
* `src/coding_practice/coding_practice.cpp`によるライブラリ

### `libcoding_practice_node`
* `libcoding_practice`ライブラリを用いたnode

# ROSのnodeのPythonでのコーディング例

# ROSのコーディングに関するTips
