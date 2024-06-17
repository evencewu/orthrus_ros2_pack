#include <iostream>
#include <memory>

struct Share
{
  int value;
};

class ClassA
{
private:
  std::shared_ptr<Share> share;

public:
  // 设置Share的shared_ptr
  void setShare(std::shared_ptr<Share> newShare)
  {
    share = newShare;
  }

  // 读取结构体Share中的值
  int readValue() const
  {
    return share->value;
  }

  // 写入新的值到结构体Share
  void writeValue(int newValue)
  {
    share->value = newValue;
  }
};

class ClassC
{
public:
  void init()
  {
    a->setShare(shared);
  }

  void update(int Value)
  {

    a->writeValue(Value);

    std::cout << "Value in Share: " << a->readValue() << " " << shared->value << std::endl; // 输出: 42
  }

  // 在main中创建std::shared_ptr<Share>
  std::shared_ptr<Share> shared = std::make_shared<Share>();

  // 创建std::shared_ptr<ClassA>
  std::shared_ptr<ClassA> a = std::make_shared<ClassA>();
};

int main()
{
  ClassC c;

  c.init();
  c.update(1);
  c.update(2);
  c.update(3);
  c.update(4);

  // 读取Share中的值

  // 由于使用了std::shared_ptr，ClassA和Share的生命周期将自动管理
  return 0;
}