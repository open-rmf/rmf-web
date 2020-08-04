import { ResourceConfigurationsType } from "../../resource-manager";

export default async function fakeAppIcons(): Promise<ResourceConfigurationsType> {
    return {
        'robots': {
            'ModelA': '/robots/magni/magni.png',
            'magni': '/robots/magni/magni.png',
        },
        'dispenser': {
            'dispenser1': '/img/'
        }
    };
}
