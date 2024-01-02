import path from "path";
import fs from "fs";
import os from "os";

const loggerDir = path.join(os.homedir(), "gsot_logs/");
let loggerActive = false;

type WriteStreamMap = {
    [key: string]: fs.WriteStream;
};
let wsMap: WriteStreamMap = {};

function getFormattedDate() {
    const date = new Date();
    const day = date.getDate().toString().padStart(2, "0");
    const month = (date.getMonth() + 1).toString().padStart(2, "0");
    const year = date.getFullYear();

    return `${month}-${day}-${year}`;
}

function initializeLogger(newPrefix: string, topics: string[]): Promise<void> {
    return new Promise((resolve, reject) => {
        if (newPrefix === "") {
            newPrefix = getFormattedDate();
        }

        const timestamp = Date.now().toString();
        const dirPath = path.join(loggerDir, newPrefix + "-" + timestamp);
        fs.mkdir(dirPath, { recursive: true }, (err) => {
            if (err) {
                reject(err);
                return;
            }
            wsMap = {};
            try {
                topics.forEach((topic) => {
                    const topicNoSlashes = topic
                        .replace(/\//g, "--")
                        .substring(2);
                    const filePath = path.join(
                        dirPath,
                        `${topicNoSlashes}.log`
                    );
                    wsMap[topicNoSlashes] = fs.createWriteStream(filePath, {
                        flags: "a",
                    });
                });
                loggerActive = true;
                resolve();
            } catch (err) {
                reject(err);
            }
        });
    });
}

function logData(data: Buffer) {
    if (!loggerActive) return;

    const timestamp = Date.now().toString();
    const dataArray = data.toString().split("\n");
    let sortedTopics = Object.keys(wsMap).sort();
    sortedTopics.shift(); // no "overview"

    console.log(`DATA ARRAY ${dataArray}`);
    console.log(`SORTED TOPICS ${sortedTopics}`);

    if (dataArray.length !== sortedTopics.length) {
        console.error("Mismatch between data buffer and topics.");
        return;
    }

    for (let i = 0; i < sortedTopics.length; i++) {
        const topic = sortedTopics[i];
        const data = dataArray[i];
        const ws = wsMap[topic];

        if (ws) ws.write(`[${timestamp}]\t${data}\n`);
    }
}

export { initializeLogger, logData };
