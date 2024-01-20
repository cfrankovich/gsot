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
    decimalPlaces: number = 2
): string {
    const timeDifference = (timestamp2 - timestamp1) / 1000;
    const formattedTime = timeDifference.toFixed(decimalPlaces);
    return `${formattedTime}s`;
}

interface ChartData {
    labels: string[];
    datasets: any[];
}

const TopicView: React.FC<TopicViewProps> = (props) => {
    const [startTime, setStartTime] = useState<number>();
    const [labels, setLabels] = useState<string[]>([]);
    const [chartData, setChartData] = useState<ChartData>({
        labels: [],
        datasets: [],
    });

    const updateStartTime = (newTime: number) => {
        if (startTime !== undefined || labels.length > 0) return;
        setStartTime(newTime);
        setLabels(["0s"]);
    };

    const updateLabels = (topicData: string) => {
        if (startTime === undefined) return;
        const newTime: number = parseInt(topicData.split("\t")[0]);
        const humanReadableTime: string = formatTimeDifference(
            startTime,
            newTime
        );
        setLabels((prevLabels) => {
            const updatedLabels = [...prevLabels, humanReadableTime];
            return updatedLabels;
        });
    };

    const updateChartData = (topicData: string) => {
        const topicDataSplit: string[] = topicData.split("\t").slice(1);

        setChartData((prevChartData) => {
            let prevData = prevChartData.datasets.slice();

            for (let i = 0; i < topicDataSplit.length; i++) {
                const singleDataValue: number = parseFloat(topicDataSplit[i]);

                if (prevData.length < topicDataSplit.length) {
                    prevData.push({
                        label: String(i),
                        data: [singleDataValue],
                        borderColor: "rgb(255, 99, 132)",
                        backgroundColor: "rgba(255, 99, 132, 0.5)",
                    });
                } else {
                    prevData[i].data.push(singleDataValue);
                }
            }

            return {
                labels: labels,
                datasets: prevData,
            };
        });
    };

    useEffect(() => {
        const getTopicData = () => {
            axios
                .get("/get-topic-data", {
                    params: {
                        topic: props.topic,
                    },
                })
                .then((res) => {
                    updateStartTime(parseInt(res.data.split("\t")[0]));
                    updateLabels(res.data);
                    updateChartData(res.data);
                })
                .catch((_err) => {
                    alert("Error reading topic.");
                });
        };

        const interval = setInterval(getTopicData, LATENCY);

        return () => clearInterval(interval);
    }, [props.topic, chartData.labels]);

    return (
        <div className="viewer">
            <LineChart data={chartData} />
        </div>
    );
};

export default TopicView;
