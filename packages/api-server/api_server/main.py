import os
import os.path
import subprocess


def main():
    result = subprocess.run(
        ['flask', 'run'],
        env={**os.environ, 'FLASK_APP': f'{os.path.dirname(__file__)}/app.py'}
    )
    exit(result.returncode)


if __name__ == '__main__':
    main()
