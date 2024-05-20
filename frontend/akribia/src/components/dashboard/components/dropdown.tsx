import { Button, Dropdown, MenuProps, Space, Tooltip, message } from "antd";
import { useContext, useState } from "react";
import Icons from "../../navbar/components/icon";
import { ReadingsContext } from "../../../contexts/readings-context";


const DropdownCustom = ()=>{

const {setDeviceId} = useContext(ReadingsContext);

const items: MenuProps['items'] = [
    {
        label: 'None',
        key: '0',
        icon: <Icons name={"radix-icons:shadow-none"}/>,
      },
    {
      label: '1st Device',
      key: '1',
      icon: <Icons name={"tabler:device-imac-search"}/>,
    },
    {
      label: '2nd Device',
      key: '2',
      icon: <Icons name={"tabler:device-imac-search"}/>,
    },
    {
      label: '3rd Device',
      key: '3',
      icon: <Icons name={"tabler:device-imac-search"}/>,
    },
    {
      label: '4rd Device',
      key: '4',
      icon: <Icons name={"tabler:device-imac-search"}/>,
    },
  ];

  const [selectedItem,setSelectedItem] = useState<string>("Select Device");
  
  const handleMenuClick: MenuProps['onClick'] = (e) => {
    if(e.key == '0'){
        setSelectedItem("Select Device");
    }
    else{
        const clickedItem = items?.[Number(e.key)]
        if (clickedItem && 'label' in clickedItem) {
          setSelectedItem(clickedItem.label as string);
        }
        setDeviceId(1);
    }
    
  };
  
  


  
  const menuProps = {
    items,
    onClick: handleMenuClick,
  };

  return(
    <Dropdown menu={menuProps}>
    <Tooltip title={"Select Device"} placement="left">    
    <Button type="default">
      <Space>
        {selectedItem}
        <Icons name="mingcute:down-fill"/>
        </Space>
    </Button>
    </Tooltip>
  </Dropdown>
  )
}

export default DropdownCustom;