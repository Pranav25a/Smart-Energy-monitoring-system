import { deserialize, serializable } from "serializr";

export class Device {
    @serializable
    public id : number;
    @serializable
    public name: string;
    
    deserialize(input: any): this {
        return Object.assign(this, deserialize(Device, input));
      }
}