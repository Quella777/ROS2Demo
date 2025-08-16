#include <iostream>
#include <thread>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <functional>            //函数包装器
#include <cpp-httplib/httplib.h> //下载相关

using namespace std;

class DownLoad
{
private:
public:
    DownLoad() = default;
    ~DownLoad() = default;

    void downLoadFile(const string &host, const string &path, function<void(const string &, const string &)> callback_word_count)
    {
        cout << "线程ID:" << this_thread::get_id() << endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if (response && response->status == 200)
        {
            callback_word_count(path, response->body);
        }
    }

    void start_download(const string &host, const string &path, function<void(const string &, const string &)> callback)
    {
        auto download_func = bind(&DownLoad::downLoadFile, this, placeholders::_1, placeholders::_2, placeholders::_3);
        thread download_thread(download_func, host, path, callback);
        download_thread.detach();
    }
};

int main()
{
    auto d = DownLoad();

    auto word_count = [](const string &path, const string &result) -> void
    {
        cout << "下载完成" << path << ":" << result.length() << "->" << result.substr(0, 9) << endl;
        // cout << result << endl;
    };

    d.start_download("http://0.0.0.0:8000", "/noval1.txt", word_count);
    d.start_download("http://0.0.0.0:8000", "/noval2.txt", word_count);
    d.start_download("http://0.0.0.0:8000", "/noval3.txt", word_count);

    this_thread::sleep_for(chrono::seconds(5)); // 等待下载完成

    return 0;
}
