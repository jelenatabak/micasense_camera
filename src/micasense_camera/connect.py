from configparser import ConfigParser
import psycopg2


def config(filename='/home/franka/catkin_ws/src/micasense_camera/src/micasense_camera/database.ini', section='postgresql'):
    """
    Read and parse the database connection parameters from database.ini file.
    """
    # Create a parser
    parser = ConfigParser()
    # Read config file
    parser.read(filename)
    # Get section, default to postgresql
    db = {}
    if parser.has_section(section):
        params = parser.items(section)
        for param in params:
            db[param[0]] = param[1]
    else:
        raise Exception('Section {0} not found in the {1} file'.format(section, filename))

    return db


def connect():
    """
    Connect to the database.
    """
    try:
        # Read connection parameters
        params = config()
        # Connect to the PostgreSQL server
        # print('Connecting to the PostgreSQL database...')
        conn = psycopg2.connect(**params)
        conn.autocommit = True
        # print('Connected to the PostgreSQL database.')
        return conn
    except (Exception, psycopg2.DatabaseError) as error:
        print(error)


def disconnect(conn):
    """
    Disconnect from the database.
    """
    if conn is not None:
        conn.close()
        # print('Database connection closed.')
    else:
        print('Invalid database connection.')
