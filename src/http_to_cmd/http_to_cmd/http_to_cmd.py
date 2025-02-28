# 写http服务器等待客户端的连接
# 收到连接后将自己的状态信息给客户端    
# 1.读取自己的电量给客户端
# 2.读取自己的机器人编号
# 3.当前状态信息
# 等待客户端的消息
# 处理客户端的消息
# 需要处理的消息
# 1.wasd控制移动将其转化为geometry_msgs消息下的cmd_vel，用if即可
# 2.暂停，需要往电机驱动增加暂停的驱动，然后自定义一个消息，对电机进行急停控制
# 1.增加自定义消息 急停
# 2.往电机控制功能包里面增加急停控制
import rclpy
from http.server import HTTPServer,BaseHTTPRequestHandler
from rclpy.node import Node
from geometry_msgs.msg import Twist


#定义一个自定义的请求处理类，继承BaseHTTPRequestHandler
class MYHTTPRequestHandler(BaseHTTPRequestHandler):
    # 处理GET请求
    def do_GET(self):
        # 打印客户端的请求路径（调试用）
        print(f"收到GET请求，路径：{self.path}")
        #发送响应状态码200，表示请求成功
        self.send_response(200)
        #设置响应头信息
        self.send_header("Content-Type","text/html; charset=utf-8")
        self.end_headers()
        #定义响应内容
        response_content = "<html><body><h1>你好，局域网的访问者！</h1></body></html>"
        #发送响应内容
        self.wfile.write(response_content.encode("utf-8"))
    #处理POST请求
    def do_POST(self):
        #获取请求头中的Content-Length 字段，确定请求体的大小
        content_length = int(self.headers.get('Content-Length', 0))
        #读取请求体的数据
        post_data = self.rfile.read(content_length).decode("utf-8")
        #打印收到的POST数据（调试用）
        print(f"收到POST数据： {post_data}")
        #发送响应状态码
        self.send_response(200)
        # 设置响应头信息
        self.send_header("Content-Type","application/json; charseturtf-8")
        self.end_headers()
        #返回json格式的响应内容
        respone_content = '{"status": "success", "message": "数据已接收"}'
        self.wfile.write(respone_content.encode("utf-8"))
    


class HttpToCmdVelNode(Node):
    def __init__(self):
        super().__init__('http_to_cmdvel_node')
        self.publisher = self.create_publisher(Twist,'app_cmd_vel',10)
        self.get_logger().info("cmd_vel 发布器已经创建")
        self.server = Node
        self.start_http_server()

    def start_http_server(self):
        host = "0.0.0.0"
        port = 8000
        # 定义服务器地址和窗口
        self.server = HTTPServer((host,port),lambda *args, **kwargs:MYHTTPRequestHandler(*args,node=self,publisher=self.publisher,**kwargs))
        print(f"服务器已启动，局域网内访问:http://192.168.206.130: {port}")

    def spin_http_server(self):
        try:
            self.server.serve_forever()
        except KeyboardInterrupt:
            self.get_logger().info("HTTP 服务器已停止")
    def stop_http_server(self):
        if self.server:
            self.server.server_close()


def main(args=Node):
    rclpy.init(args=args)
    node = HttpToCmdVelNode()

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n服务器已停止")
        server.server_close()
