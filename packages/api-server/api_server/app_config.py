class AppConfig():
    def __init__(self, dict):
        self.db_url: str = dict['db_url']
        self.static_path: str = dict['static_path']
        self.static_directory: str = dict['static_directory']
