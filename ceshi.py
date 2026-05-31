def function():
    print("这是一个测试函数")
    def inner_function():
        print("这是一个内部函数")
    return inner_function

function()

class Test:
    a="yellow"
    def method():
        print("这是一个测试方法")

Test.method()
print(Test.a)

