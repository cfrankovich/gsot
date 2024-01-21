import React, { useEffect, useState } from "react";
import axios from "axios";
import LineChart from "./LineChart";

const LATENCY: number = 1000;
type TopicViewProps = {
    topic: string;
};

function formatTimeDifference(
    timestamp1: number,
    timestamp2: number,
    decimalPlaces: number = 0
): string {
    const timeDifference = (timestamp2 - timestamp1) / 1000;
    const formattedTime = timeDifference.toFixed(decimalPlaces);
    return `${formattedTime}s`;
}

interface ChartData {
    labels: string[];
    datasets: any[];
}

let red = 255;
let blue = 132;

const TopicView: React.FC<TopicViewProps> = (props) => {
    const [startTime, setStartTime] = useState<number>();
    const [labels, setLabels] = useState<string[]>([]);
    const [chartData, setChartData] = useState<ChartData>({
        labels: [],
        datasets: [],
    });
    const [loggerStatus, setLoggerStatus] = useState<Boolean>(false);

    const updateStartTime = (newTime: number) => {
        if (startTime !== undefined || labels.length > 0) return;
        setStartTime(newTime);
        setLabels(["0s"]);
    };

    useEffect(() => {
        axios
            .get("/logger-status")
            .then((res) => {
                setLoggerStatus(res.data);
            })
            .catch((_err) => {
                console.log("Error fetching logger status.");
            });
    }, []);

    const updateLabels = (topicData: string, startTime: number) => {
        const newTime = parseInt(topicData.split("\t")[0]);
        const humanReadableTime = formatTimeDifference(startTime, newTime);
        return humanReadableTime;
    };

    const updateChartData = (
        topicData: string,
        prevChartData: any,
        labels: string[]
    ) => {
        const topicDataSplit = topicData.split("\t").slice(1);
        let prevData = prevChartData.datasets.slice();

        for (let i = 0; i < topicDataSplit.length; i++) {
            const singleDataValue = parseFloat(topicDataSplit[i]);

            if (prevData.length < topicDataSplit.length) {
                prevData.push({
                    label: String(i),
                    data: [singleDataValue],
                    borderColor: `rgb(${red}, 99, ${blue})`,
                    backgroundColor: `rgba(${red}, 99, ${blue}, 0.5)`,
                });
                red -= 50;
                blue += 50;
            } else {
                prevData[i].data.push(singleDataValue);
            }
        }
        red = 255;
        blue = 132;

        return {
            labels: labels,
            datasets: prevData,
        };
    };

    useEffect(() => {
        const getTopicData = () => {
            axios
                .get("/get-all-topic-data", {
                    params: { topic: props.topic },
                })
                .then((res) => {
                    const allData = res.data;
                    const initialTime = parseInt(allData[0].split("\t")[0]);
                    setStartTime(initialTime);

                    let newLabels: string[] = [];
                    let newChartData: any = { labels: [], datasets: [] };

                    allData.forEach((dataElement: string) => {
                        newLabels.push(updateLabels(dataElement, initialTime));
                        newChartData = updateChartData(
                            dataElement,
                            newChartData,
                            newLabels
                        );
                    });

                    setLabels(newLabels);
                    setChartData(newChartData);
                })
                .catch((_err) => {
                    alert("Error reading topic.");
                });
        };

        const interval = setInterval(getTopicData, LATENCY);

        return () => clearInterval(interval);
    }, [props.topic]);

    return (
        <div className="viewer">
            {!loggerStatus && <p>Logger is not active.</p>}
            {loggerStatus && <LineChart data={chartData} />}
        </div>
    );
};

export default TopicView;
