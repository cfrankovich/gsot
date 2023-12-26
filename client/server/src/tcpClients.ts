import { Socket, createConnection } from "net";
import { config } from "process";
const net = require("net");

const METRIC_TCP_PORT = 12301;
const CONFIG_TCP_PORT = 12303;
let configSocket: Socket | null = null;

function startMetricDataTCPServer() {
    const tcpServer = net.createServer((socket: Socket) => {
        console.log("Metric Data TCP Client Connected");

        socket.on("data", (data) => {
            processMetricData(data);
        });

        socket.on("end", () => {
            console.log("Metric Data TCP Client Disconnected");
        });
    });

    tcpServer.listen(METRIC_TCP_PORT, () => {
        console.log(
            `Metric Data TCP Server listening on port ${METRIC_TCP_PORT}`
        );
    });
}

function processMetricData(data: Buffer) {}

function startConfigTCPServer() {
    const tcpServer = net.createServer((socket: Socket) => {
        console.log("Config TCP Client Connected");
        configSocket = socket;

        socket.on("end", () => {
            console.log("Config TCP Client Disconnected");
        });
    });

    tcpServer.listen(CONFIG_TCP_PORT, () => {
        console.log(`Config TCP Server listening on port ${CONFIG_TCP_PORT}`);
    });
}

function sendConfigRequest(message: string): Promise<string> {
    return new Promise((resolve, reject) => {
        if (configSocket) {
            const timeout = setTimeout(() => {
                configSocket?.removeAllListeners();
                reject("Config request timed out");
            }, 5000); // 5 second timeout

            configSocket.write(message);

            configSocket.once("data", (data) => {
                clearTimeout(timeout);
                resolve(data.toString());
                configSocket?.removeAllListeners("error");
            });

            configSocket.on("error", (err) => {
                clearTimeout(timeout);
                console.error("Config TCP Client Eerror: ", err);
                configSocket?.removeAllListeners("data");
                reject(err);
            });
        } else {
            reject("Config socket not established");
        }
    });
}

export { startMetricDataTCPServer, startConfigTCPServer, sendConfigRequest };
