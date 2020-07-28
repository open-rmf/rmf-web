import { IconContextType } from "../../icons-manager";

export default async function fakeAppIcons(): Promise<IconContextType> {
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
