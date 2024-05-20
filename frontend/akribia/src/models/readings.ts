import { deserialize, serializable } from "serializr";

export class Readings {
  @serializable
  public id:number;
  @serializable
  public voltage1: number;
  @serializable
  public voltage2: number;
  @serializable
  public voltage3: number;
  @serializable
  public current1: number;
  @serializable
  public current2: number;
  @serializable
  public current3: number;
  @serializable
  public frequency1: number;
  @serializable
  public frequency2: number;
  @serializable
  public frequency3: number;
  @serializable
  public power1: number;
  @serializable
  public power2: number;
  @serializable
  public power3: number;
  @serializable
  public sequence: boolean;
  @serializable
  public deviceId: number;
  @serializable
  public createdAt: Date;

  deserialize(input: any): this {
    return Object.assign(this, deserialize(Readings, input));
  }
}
