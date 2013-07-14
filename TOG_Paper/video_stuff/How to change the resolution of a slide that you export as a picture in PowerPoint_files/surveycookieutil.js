if (!window.MS)
{
	MS = {};
}

if (!MS.Support)
{
	MS.Support = {};
}

if (!MS.Support.Fms)
{
	MS.Support.Fms = {};
}

if (!MS.Support.Fms.CookieUtil)
{
	MS.Support.Fms.CookieUtil = {

		getCookie: function(key)
		{
			var entities = document.cookie.split(";");
			for (var i = 0; i < entities.length; ++i)
			{
				var j = entities[i].indexOf("=");
				var s = entities[i].substring(0, j);
				if (s != "" && (s == key || s == " " + key))
				{
					return entities[i].substring(j + 1);
				}
			}
			return null;
		},

		setCookie: function(key, value, expires, domain, path)
		{
			domain = domain || (typeof(gCookieDomain) != "undefined" ? (gCookieDomain || document.domain) : document.domain);
			path = path || "/";
			document.cookie = key + "=" + value + "; domain=" + domain + "; path=" + path + "; " + (expires ? ("expires=" + expires.toGMTString() + ";") : "");
		},

		setSessionCookie: function(key, value, domain, path)
		{
			this.setCookie(key, value, null, domain, path);
		},

		removeCookie: function(key, domain, path)
		{
			this.setCookie(key, "", (new Date(0, 0, 0)), domain, path);
		},

		//key=subkey1=value1&subkey2=value2
		getSubCookie: function(key, subkey)
		{
			var value = this.getCookie(key);
			if (value)
			{
				var entities = value.split("&");
				for (var i = 0; i < entities.length; ++i)
				{
					var j = entities[i].indexOf("=");
					var s = entities[i].substring(0, j);
					if (s != "" && (s == subkey))
					{
						return entities[i].substring(j + 1);
					}
				}
			}
			return null;
		},

		setSubCookie: function(key, subkey, subvalue, expires, domain, path)
		{
			var value = this.getCookie(key);
			if (!value)
			{
				value = subkey + '=' + subvalue;
			}
			else
			{
				var cookiearray = value.split('&');
				var i = 0;
				for (; i < cookiearray.length; i++)
				{
					value = cookiearray[i];
					var cookiename = value.substring(0, value.indexOf('='));
					if (subkey == cookiename)
					{
						cookiearray[i] = cookiename + '=' + subvalue;
						break;
					}
				}
				if (i >= cookiearray.length)
				{
					cookiearray[i] = subkey + '=' + subvalue;
				}
				value = cookiearray.join('&');
			}
			this.setCookie(key, value, expires, domain, path);
		}
	}
}