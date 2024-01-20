import React, { useState } from "react";
import { Line } from "react-chartjs-2";
import Chart from "chart.js/auto";
import { CategoryScale } from "chart.js";

type LineChartProps = {
    data: any;
};

Chart.register(CategoryScale);

const options = {};

const LineChart: React.FC<LineChartProps> = (props) => {
    return (
        <div className="chart-container">
            <Line options={options} data={props.data} />
        </div>
    );
};
export default LineChart;
