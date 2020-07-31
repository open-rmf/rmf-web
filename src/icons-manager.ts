export type IconConfigurationsType = Record<string, Record<string, string>>
const axios = require('axios').default;
export default class IconManager {

    static getIconsConfigurationFile = async (): Promise<IconConfigurationsType> => {
        try {
            const response = await axios.get('/assets/icons/icons.json')
            return response.data as IconConfigurationsType;
        } catch (error) {
            console.error(error)
            return {}
        }
    }

    static getRobotIcon(icons: IconConfigurationsType, model: string): string | null {
        return icons.robots ? `/assets/icons${IconManager.getIcon(icons.robots, model)}` : null;
    }

    private static getIcon(icons: Record<string, string>, key: string): string | null {
        return icons.hasOwnProperty(key) ? icons[key] : null;
    }
}
