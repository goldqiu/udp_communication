#include "udp_communication/node_flow/message_send_flow.hpp"

using namespace udp_communication;

typedef struct sockaddr* saddrp;
int sockfd;
struct sockaddr_in addr = {};
socklen_t addr_len;
std::string buf,str;


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"message_send_node");
    ros::NodeHandle nh("~");
    std::string buf2 = "hello_ausim"; //创建接收数据的数组

    std::shared_ptr<MessageSendFlow> message_send_flow_ptr = std::make_shared<MessageSendFlow>(nh);

    sockfd = socket(AF_INET,SOCK_DGRAM,0);
    if (0 > sockfd)
    {
        perror("socket");
        return -1;
    }
    //绑定ip和端口号
    addr.sin_family = AF_INET;
    addr.sin_port = htons(12345);
    addr.sin_addr.s_addr = inet_addr("10.42.0.1 ");
    addr_len = sizeof(struct sockaddr_in);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        message_send_flow_ptr->Run();

        sendto(sockfd, buf2.data(),buf2.size(), 0, (saddrp)&addr,sizeof(addr));

        rate.sleep();
    }

    close(sockfd);
    return 0;

}

