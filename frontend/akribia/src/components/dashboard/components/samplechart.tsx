import React, { useContext } from 'react';
import { 
  Chart as ChartJS, 
  CategoryScale, 
  LinearScale, 
  PointElement, 
  LineElement, 
  Filler, 
  Tooltip, 
  Legend,
  TimeScale,
  TimeSeriesScale 
} from 'chart.js';
import 'chartjs-adapter-date-fns'; // Use the date-fns adapter for better date handling
import { Line } from 'react-chartjs-2';
import { ReadingsContext } from '../../../contexts/readings-context';
import zoomPlugin from 'chartjs-plugin-zoom';

// Register the chart components
ChartJS.register(
  CategoryScale, 
  LinearScale, 
  PointElement, 
  LineElement, 
  Filler, 
  Tooltip, 
  Legend,
  TimeScale,
  TimeSeriesScale,
  zoomPlugin
);

const AreaChart = ({ label }) => {
    const { readings } = useContext(ReadingsContext);
    const voltage = ["voltage1,voltage2,voltage3"];
    const current = ["current1,current2,current3"];
    const power = ["power1,power2,power3"];
    const frequency = ["frequency1,frequency2,frequency3"];
    const datasets = [
        {
            bgc: 'rgba(75,192,192,0.2)',
            bdc: 'rgba(75,192,192,1)',
        },
        {
            bgc: 'rgba(255,99,132,0.2)',
            bdc: 'rgba(255,99,132,1)',
        },
        {
            bgc: 'rgba(54, 162, 235, 0.2)',
            bdc: 'rgba(54, 162, 235, 1)',
        },
        {
            bgc: 'rgba(255, 206, 86, 0.2)',
            bdc: 'rgba(255, 206, 86, 1)',
        }];
    // Convert readings to chart data
    const data = {
        labels: readings?.map(reading => new Date(reading.createdAt)),
        datasets: [
            {
                label: label,
                data: readings?.map(reading => reading[label]),
                fill: true,
                backgroundColor: `${label[0] == 'v' ? datasets[0].bgc : label[0] == 'c' ? datasets[1].bgc : label[0] == 'p' ? datasets[2].bgc : label[0] == 'f' ? datasets[3].bgc : datasets[0].bgc }`,
                borderColor: `${label[0] == 'v' ? datasets[0].bdc : label[0] == 'c' ? datasets[1].bdc : label[0] == 'p' ? datasets[2].bdc : label[0] == 'f' ? datasets[3].bdc : datasets[0].bdc }`,
            },
        ],
    };

    // Chart options
    const options = {
        responsive: true,
        scales: {
            y: {
                beginAtZero: true,
            },
            x: {
                type: 'time', // Use a time scale
                time: {
                    unit: `minute`, // Set your own time unit (minute, hour, day, etc.)
                    tooltipFormat: 'PPpp', // Define how you want the tooltip date to appear
                },
                ticks: {
                    autoSkip: true, // Only show a subset of labels to avoid overlap
                    maxRotation: 0, // Avoid rotation of labels
                    maxTicksLimit: 6 // Limit the number of labels displayed
                }
            }
        },
        plugins: {
            zoom: {
                pan: {
                  enabled: true,
                  mode: 'x',
                },
                zoom: {
                  wheel: {
                    enabled: true,
                  },
                  pinch: {
                    enabled: true,
                  },
                  mode: 'x',
                },},
            legend: {
                display: true, // If you want to show the legend
            },
            tooltip: {
                mode: 'index',
                intersect: false,
            },
        },
        interaction: {
            mode: 'nearest',
            axis: 'x',
            intersect: false
        },
        maintainAspectRatio: false, // To make the chart responsive in terms of its height as well
    };

    return <Line data={data} options={options} />;
};

export default AreaChart;