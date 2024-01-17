import React, { useEffect, useState } from "react";
import axios from "axios";

const LATENCY: number = 1000;
type TopicViewProps = {
    topic: string;
};

const TopicView: React.FC<TopicViewProps> = (props) => {
    const [topicData, setTopicData] = useState<string>("");
    useEffect(() => {
        const getTopicData = () => {
            axios
                .get("/get-topic-data", {
                    params: {
                        topic: props.topic,
                    },
                })
                .then((res) => {
                    setTopicData(res.data);
                })
                .catch((_err) => {
                    setTopicData("");
                    alert("Error reading topic.");
                });
        };

        const interval = setInterval(getTopicData, LATENCY);

        return () => clearInterval(interval);
    }, [props.topic]);

    return (
        <div className="viewer">
            <p>{topicData}</p>
        </div>
    );
};

export default TopicView;
