import path from "path";
import fs from "fs";
import os from "os";
const readline = require("readline");

const loggerDir = path.join(os.homedir(), "gsot_logs/");
let loggerActive = false;
let dirPath = "";

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

function readLastLineOfFile(filePath: string): Promise<string> {
    return new Promise((resolve, reject) => {
        const readStream = fs.createReadStream(filePath);

        const rl = readline.createInterface({
            input: readStream,
            crlfDelay: Infinity,
        });

        let lastLine = "";

        rl.on("line", (line: string) => {
            lastLine = line;
        });

        rl.on("close", () => {
            resolve(lastLine);
        });

        rl.on("error", (error: Error) => {
            reject(error);
        });
    });
}

function readAllLines(filePath: string): Promise<string[]> {
    return new Promise((resolve, reject) => {
        const readStream = fs.createReadStream(filePath);

        const rl = readline.createInterface({
            input: readStream,
            crlfDelay: Infinity,
        });

        let lines: string[] = [];

        rl.on("line", (line: string) => {
            lines.push(line);
        });

        rl.on("close", () => {
            resolve(lines);
        });

        rl.on("error", (error: Error) => {
            reject(error);
        });
    });
}

async function getTopicData(topic: string): Promise<string> {
    if (!loggerActive || topic === undefined) {
        return new Promise((_resolve, reject) => {
            reject("Logger is not active or topic is undefined.");
        });
    }
    const topicNoSlashes = topic.replace(/\//g, "--");

    if (wsMap[topicNoSlashes] === undefined) {
        return new Promise((_resolve, reject) => {
            reject("Topic not found in write stream map.");
        });
    }

    try {
        const filePath = path.join(dirPath, `${topicNoSlashes}.log`);
        const lastLine = await readLastLineOfFile(filePath);
        return new Promise((resolve, _reject) => {
            resolve(lastLine);
        });
    } catch (error) {
        return new Promise((_resolve, reject) => {
            reject("Error reading log file.");
        });
    }
}

function initializeLogger(newPrefix: string, topics: string[]): Promise<void> {
    return new Promise((resolve, reject) => {
        if (newPrefix === "") {
            newPrefix = getFormattedDate();
        }

        const timestamp = Date.now().toString();
        dirPath = path.join(loggerDir, newPrefix + "-" + timestamp);
        fs.mkdir(dirPath, { recursive: true }, (err) => {
            if (err) {
                dirPath = "";
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
                dirPath = "";
                wsMap = {};
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

    if (dataArray.length !== sortedTopics.length) {
        console.error("Mismatch between data buffer and topics.");
        return;
    }

    for (let i = 0; i < sortedTopics.length; i++) {
        const topic = sortedTopics[i];
        const data = dataArray[i];
        const ws = wsMap[topic];

        if (ws) ws.write(`${timestamp}\t${data}\n`);
    }
}

function stopLogger() {
    loggerActive = false;

    Object.values(wsMap).forEach((ws) => {
        ws.end(); // Close the write streams
    });

    dirPath = "";
    wsMap = {};
}

function getLoggerStatus(): Boolean {
    return loggerActive;
}

async function getAllTopicData(topic: string): Promise<string[]> {
    if (!loggerActive || topic === undefined) {
        return new Promise((_resolve, reject) => {
            reject("Logger is not active or topic is undefined.");
        });
    }
    const topicNoSlashes = topic.replace(/\//g, "--");

    if (wsMap[topicNoSlashes] === undefined) {
        return new Promise((_resolve, reject) => {
            reject("Topic not found in write stream map.");
        });
    }

    try {
        const filePath = path.join(dirPath, `${topicNoSlashes}.log`);
        const lines: string[] = await readAllLines(filePath);
        let data: string[] = [];
        lines.forEach((line: string) => {
            data.push(line);
        });
        return new Promise((resolve, _reject) => {
            resolve(data);
        });
    } catch (error) {
        return new Promise((_resolve, reject) => {
            reject("Error reading log file.");
        });
    }
}

export {
    initializeLogger,
    logData,
    getTopicData,
    stopLogger,
    getLoggerStatus,
    getAllTopicData,
};
