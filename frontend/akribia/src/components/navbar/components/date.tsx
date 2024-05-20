import React, { useContext } from 'react';
import { DatePicker, DatePickerProps, GetProps, Space } from 'antd';
import { ReadingsContext } from '../../../contexts/readings-context';
const DateComponent: React.FC = () => {
  const { RangePicker } = DatePicker;
  const { setFromDate, setEndDate } = useContext(ReadingsContext);
  type RangePickerProps = GetProps<typeof DatePicker.RangePicker>;
  const onChange = (
    value: DatePickerProps['value'] | RangePickerProps['value'],
    dateString: [string, string] | string,
  ) => {
    setFromDate(dateString?.[0]);
    setEndDate(dateString?.[1])
  };
  return (
    <Space direction="vertical" size={12}>
      <RangePicker
        showTime={{ format: 'HH:mm:ss' }}
        format="YYYY-MM-DDTHH:mm:ss"
        onChange={onChange}
      />
    </Space>
  );
};

export default DateComponent;
