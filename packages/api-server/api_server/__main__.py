import uvicorn
from api_server.app import app


def main():
    uvicorn.run(app, host="127.0.0.1", port=8000)


if __name__ == '__main__':
    main()
