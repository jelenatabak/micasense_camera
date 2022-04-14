
from sshtunnel import SSHTunnelForwarder

def create_SSH_tunnel():

    remote_host = '161.53.68.231'
    remote_port = 22
    remote_user = 'agrosparc_user'
    remote_user_pass = 'vege123!'

    local_host = 'localhost'
    local_port = 5432

    server = SSHTunnelForwarder(
       ssh_address_or_host=(remote_host, remote_port),
       ssh_username=remote_user,
       ssh_password=remote_user_pass,
       remote_bind_address=('localhost', 5432),
       local_bind_address=(local_host, local_port),)

    return server
