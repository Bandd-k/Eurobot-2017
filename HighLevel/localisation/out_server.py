import vk_api

def init_vk():
    login, password = 'banddk1@gmail.com',                                                                                                                                  'boroda94'

    vk_session = vk_api.VkApi(login, password)
    try:
        vk_session.authorization()
    except vk_api.AuthorizationError as error_msg:
        print(error_msg)
        return

    vk = vk_session.get_api()

    response = vk.messages.send(user_id =22100673 ,message="hi")