import pandas as pd
from psycopg2 import DataError, InternalError, DatabaseError, ProgrammingError

def delete_camera_trigger(conn):
    """
    Deleting camera records
    :param conn: Postgres connector
    :return: cameras as a pandas dataframe
    """

    # Create query
    timeout = 'SET statement_timeout = 999; '
    query = 'DELETE FROM camera_trigger'
    # Obtain data as a Pandas dataframe
    try:
        cursor = conn.cursor()
        cursor.execute(timeout+query)
        conn.commit()
    except (DataError, DatabaseError, ProgrammingError):
        print('Incorrect database query sent: fetch_cameras.')
        conn.rollback()
    except InternalError:
        print('Rollback needed due to previous error.')
        conn.rollback()

def insert_camera_trigger(insert_values, conn):
    """
    Inserting camera data into the database
    :param insert_values: list of values for insert (name, description, capture)
    :param conn: Postgres connector
    """

    cur = conn.cursor()
    timeout = 'SET statement_timeout = 25000; '

    # Create and execute query and check the status
    #try:
    values_template = ','.join(['%s'] * len(insert_values))
    query = 'INSERT INTO camera_trigger (' \
            'timestamp, ' \
            'trigger, ' \
            'source, ' \
            'device_id, ' \
            'zone_id, ' \
            'plant_id) VALUES ({})'.format(values_template)

    cur.execute(timeout+query, insert_values)
    conn.commit()
    # Disconnect from the 3Smart database
    cur.close()
    # except (DataError, DatabaseError, ProgrammingError):
    #     print('Incorrect database query sent: insert_cameras.')
    #     cur.statusmessage
    #     conn.rollback()
    # except InternalError:
    #     print('Rollback needed due to previous error.')
    #     cur.statusmessage
    #     conn.rollback()

def update_camera_trigger(trigger, source, timestamp, conn):
    """
    Updating camera data into the database
    :param trigger: new trigger data
    :param source: new source data
    :param timestamp: timestamp of record to update
    :param conn: Postgres connector
    """

    cur = conn.cursor()
    timeout = 'SET statement_timeout = 25000; '

    # Create and execute query and check the status
    #try:
    query = "UPDATE camera_trigger \
             SET trigger = {}, source = '{}' \
             WHERE timestamp = '{}'".format(trigger, source, timestamp)

    cur.execute(timeout+query)
    conn.commit()
    # Disconnect from the 3Smart database
    cur.close()
    # except (DataError, DatabaseError, ProgrammingError):
    #     print('Incorrect database query sent: insert_cameras.')
    #     cur.statusmessage
    #     conn.rollback()
    # except InternalError:
    #     print('Rollback needed due to previous error.')
    #     cur.statusmessage
    #     conn.rollback()

def fetch_camera_trigger(conn):
    """
    Fetching of all the chamber IDs from the database.
    :param conn: Postgres connector
    :return: cameras as a pandas dataframe
    """

    # Create query
    timeout = 'SET statement_timeout = 999; '
    query = 'SELECT * FROM camera_trigger'
    # Obtain data as a Pandas dataframe
    try:
        cameras = pd.read_sql_query(timeout+query, conn)
        return cameras
    except (DataError, DatabaseError, ProgrammingError):
        print('Incorrect database query sent: fetch_cameras.')
        conn.rollback()
    except InternalError:
        print('Rollback needed due to previous error.')
        conn.rollback()


def fetch_camera_trigger_history(conn):
    """
    Fetching of all the chamber IDs from the database.
    :param conn: Postgres connector
    :return: cameras as a pandas dataframe
    """

    # Create query
    timeout = 'SET statement_timeout = 999; '
    query = 'SELECT * FROM camera_trigger_history'
    # Obtain data as a Pandas dataframe
    try:
        cameras = pd.read_sql_query(timeout+query, conn)
        return cameras
    except (DataError, DatabaseError, ProgrammingError):
        print('Incorrect database query sent: fetch_cameras.')
        conn.rollback()
    except InternalError:
        print('Rollback needed due to previous error.')
        conn.rollback()


def fetch_camera_measurements(conn):
    """
    Fetching of all the chamber IDs from the database.
    :param conn: Postgres connector
    :return: cam_meas as a pandas dataframe
    """

    # Create query
    timeout = 'SET statement_timeout = 999; '
    query = 'SELECT * FROM camera_measurements'
    # Obtain data as a Pandas dataframe
    try:
        cam_meas = pd.read_sql_query(timeout+query, conn)
        return cam_meas
    except (DataError, DatabaseError, ProgrammingError):
        print('Incorrect database query sent: fetch_camera_measurements.')
        conn.rollback()
    except InternalError:
        print('Rollback needed due to previous error.')
        conn.rollback()


def insert_camera_measurements(insert_values, conn):
    """
    Inserting telemetry data into the database
    :param insert_values: list of values for insert (batch_timestamp, timestamp, biomass, lai, ndvi, gndvi, rvi, sr,
    pri, ci, plant_id, zone_id, device_id, image_path)
    :param conn: Postgres connector
    """

    cur = conn.cursor()
    timeout = 'SET statement_timeout = 25000; '

    # Create and execute query and check the status
    #try:
    values_template = ','.join(['%s'] * len(insert_values))
    query = 'INSERT INTO camera_measurements (' \
            'batch_timestamp, ' \
            'timestamp, ' \
            'ndvi, ' \
            'sr, ' \
            'gndvi, ' \
            'osavi, ' \
            'msavi, ' \
            'mtvi2, ' \
            'savi, ' \
            'ndre, ' \
            'evi, ' \
            'tvi, ' \
            'rdvi, ' \
            'dvi, ' \
            'cir, ' \
            'plant_id, ' \
            'zone_id, ' \
            'device_id, ' \
            'image_path) VALUES ({})'.format(values_template)
    cur.execute(timeout+query, insert_values)
    conn.commit()
    # Disconnect from the 3Smart database
    cur.close()
    # except (DataError, DatabaseError, ProgrammingError):
    #     print('Incorrect database query sent: insert_camera_measurements.')
    #     cur.statusmessage
    #     conn.rollback()
    # except InternalError:
    #     print('Rollback needed due to previous error.')
    #     cur.statusmessage
    #     conn.rollback()
