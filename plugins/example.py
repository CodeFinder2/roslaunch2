import Pyro4

class ExamplePlugin:
    def __init__(self):
        pass

    @staticmethod
    @Pyro4.expose
    def static_method():
        print("Hello World!")

    @Pyro4.expose
    def method():
        print("Hello Members!")
