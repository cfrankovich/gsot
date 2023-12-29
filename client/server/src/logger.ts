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
        });

        wsMap = {};
        try {
            topics.forEach((topic) => {
                const topicNoSlashes = topic.replace("/", "-"); // either this or sub directories
                const filePath = path.join(
                    dirPath,
                    `${topicNoSlashes}-${timestamp}.log`
                );
                wsMap[filePath] = fs.createWriteStream(filePath, {
                    flags: "a",
                });
            });

            loggerActive = true;
            resolve();
        } catch (err) {
            reject(err);
        }
    });
}

function logData(data: Buffer) {
    if (!loggerActive) return;
    // TODO: log data
}

export { initializeLogger, logData };
