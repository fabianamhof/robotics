from pymongo import MongoClient


class Database(object):
    domain = "localhost"
    port = 27017
    timeout = 5000
    user = "robotics_admin"
    password = "Robotics123"
    database = None

    @staticmethod
    def instantiate_mongo_client():
        client = MongoClient(
            host=[str(Database.domain) + ":" + str(Database.port)],
            serverSelectionTimeoutMS=Database.timeout,
            username=Database.user,
            password=Database.password
        )
        Database.database = client["RBT"]

    @staticmethod
    def load_basic_tree():
        collection = Database.database["basic_tree"]
        if collection.count() == 0:
            return None
        return collection.find()

    @staticmethod
    def insert_basic_tree(basic_tree):
        collection = Database.database["basic_tree"]
        collection.insert(basic_tree)

    @staticmethod
    def load_sub_tree(name):
        collection = Database.database[name]
        if collection.count() == 0:
            return None
        return collection.find()

    @staticmethod
    def insert_sub_tree(name, sub_tree):
        collection = Database.database[name]
        collection.insert(sub_tree)


