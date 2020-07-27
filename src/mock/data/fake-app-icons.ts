import { IconContextType } from "../../icons-manager";

export default async function fakeAppIcons(): Promise<IconContextType> {
    return {
        'robot': {
            'ModelA': '/assets/icons/robots/magni/magni.png',
            'magni': '/assets/icons/robots/magni/magni.png',
        },
        'dispenser': {
            'dispenser1': '/img/'
        }
    };
}
