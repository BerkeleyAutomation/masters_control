import socket, os, logging
from multiprocessing import Process
from SocketServer import TCPServer
from BaseHTTPServer import BaseHTTPRequestHandler
from urlparse import urlparse, parse_qs
from joblib import load
from perception import write_video
from time import sleep

logging.getLogger().setLevel(logging.INFO)

def save_video(trial_path, fps):
    logging.info("Saving video at {0}".format(trial_path))

    webcam_data_path = os.path.join(trial_path, 'webcam')
    webcam_finished_flag = os.path.join(webcam_data_path, '.finished')
    while not os.path.isfile(webcam_finished_flag):
        sleep(1e-2)

    all_chunks = os.listdir(webcam_data_path)
    all_chunks.sort()

    webcam_data = []
    for i in range(1, len(all_chunks)):
        webcam_data.extend(load(os.path.join(webcam_data_path, '{}.jb'.format(i))))

    frames = [data[1] for data in webcam_data]
    write_video(frames, os.path.join(trial_path, 'webcam.avi'), fps=fps)
    logging.info("Finished saving video!")

class MyTCPServer(TCPServer):
    def server_bind(self):
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.server_address)

class MyHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        logging.getLogger().setLevel(logging.INFO)

        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)

        if parsed.path == '/render_video':
            trial_path = query['trial_path'][0]
            fps = int(query['fps'][0])
            if os.path.exists(trial_path):
                p = Process(target=save_video, args=(trial_path,fps))
                p.start()
                self.send_response(200)

        self.send_response(404)

httpd = MyTCPServer(("", 1050), MyHandler)
logging.info("Serving video service!")
httpd.serve_forever()
