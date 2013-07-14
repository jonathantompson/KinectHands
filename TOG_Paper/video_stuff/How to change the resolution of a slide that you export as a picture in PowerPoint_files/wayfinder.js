/*globals $*/
/// <reference path="http://ajax.aspnetcdn.com/ajax/jQuery/jquery-1.7.2-vsdoc.js"/>

var Microsoft = window.Microsoft || {};
Microsoft.Support = Microsoft.Support || {};
Microsoft.Support.GSS = Microsoft.Support.GSS || {};

Microsoft.Support.GSS.WayFinder = function (container, args) {
    // using
    var ScopeNodeType = Microsoft.Support.GSS.ScopeNodeType;
    var ScopeNode = Microsoft.Support.GSS.ScopeNode;

    var _this = this;
    var _$this = $(this);
    var _$container = $(container);
    var _defaultOptions = {
        enable: true,
        language: 'en-us',
        isRTL: false,
        path: '',
        isFullTopicPath: false,
        topicKeys: []
    };
    var _opts = $.extend(true, {}, _defaultOptions, args);
    var _$modalities = $(".modalities li", _$container);
    var _$productTiles = $(".products ul:first > li.product-tile", _$container);
    var _$products = $(".products li.product-item", _$container);
    var _$productMenus = $("ul", _$productTiles);

    var _$curModality = _$modalities.filter(".selected");
    var _$curTile = _$productTiles.filter(".selected");
    var _$curProduct = _$curTile;

    var _curModalityData = null;
    var _rootNode = new ScopeNode(ScopeNodeType.Root);
    var _curScopeNode = null;
    var _curProductNode = null;

    var _parsePath = ScopeNode.prototype.parsePath;

    var _tidyPath = function (path) {
        return _parsePath(path).join("/");
    };

    var _changeProduct = function (e, $item) {
        var oldScopeNode = _curScopeNode;
        _curScopeNode = _curProductNode = $item.data("node");
        _updateModalityLinks();
        var preventEvent = _$this.triggerHandler("productselect", [_curProductNode, e]) === false;

        if (oldScopeNode !== _curScopeNode) {
            if (_$this.triggerHandler("scopechange", [_curScopeNode, oldScopeNode]) === false) {
                preventEvent = true;
            }
        }

        if (preventEvent) {
            e.preventDefault();
        }
    };

    var _getModalityData = function ($item) {
        return {
            type: $item.data('type'),
            rootPath: $item.data('root-path')
        };
    };

    var _getProductNode = function ($item) {
        var node = $item.data("node");
        if (!(node instanceof ScopeNode)) {
            node = new ScopeNode(
                ScopeNodeType.Product,
                $item.data("key"),
                $.trim($item.find("> a").text()),
                {
                    seamId: $item.data('seam-id'),
                    productId: $item.data("product-id")
                }
            );
            $item.data("node", node);
        }
        return node;
    };

    var _getHref = function () {    //params string[] paths
        var parts = [];
        var paths = [];
        $.each(arguments, function (idx, path) {
            if (path) {
                if ($.isArray(path)) {
                    paths = paths.concat(path);
                } else {
                    paths.push(path);
                }
            }
        });
        $.each(paths, function (idx, path) {
            path = $.trim(path);
            if (path) {
                $.each(path.split(/[\\\/]/), function (idx, part) {
                    part = $.trim(part);
                    if (part) {
                        parts.push(part);
                    }
                });
            }
        });
        return '/' + (parts.length > 0 ? parts.join('/') + '/' : '');
    };

    var _updateModalityLinks = function () {
        $.each(_$modalities, function () {
            var $this = $(this);
            var data = _getModalityData($this);
            $this.find("a").attr("href", _getHref(data.rootPath, _curScopeNode ? _curScopeNode.getPath() : ''));
        });
    };

    var _updateProductLinks = function () {
        $.each(_$products, function () {
            var $this = $(this);
            $this.find("a").attr("href", _getHref(_curModalityData.rootPath, $this.data("node").getPath()));
        });
    };

    var _init = function () {
        // set current modality
        _curModalityData = _getModalityData(_$curModality);

        // build product nodes structure
        _rootNode = new ScopeNode(ScopeNodeType.Root);
        _curProductNode = null;
        _$productTiles.each(function () {
            var $tile = $(this);
            var tileNode = _getProductNode($tile);
            _rootNode.addChildren(tileNode);
            $tile.find("li.product-item").each(function () {
                var $child = $(this);
                var childNode = _getProductNode($child);
                tileNode.addChildren(childNode);

            });

            // set current selected product node
            if (_curProductNode === null && $tile.is(".selected")) {
                _curProductNode = $tile.find("li.product-item.selected").data("node") || tileNode;
            }
        });

        // setup init scope path nodes(we had to make fake topic nodes because topic nodes are not loaded in wayfinder)
        _curScopeNode = _curProductNode;
        if (_curScopeNode && $.isArray(_opts.topicKeys)) {
            $.each(_opts.topicKeys, function (idx, topicKey) {
                if (topicKey) {
                    var fakeTopicNode = new ScopeNode(ScopeNodeType.Topic, topicKey);
                    fakeTopicNode.setParent(_curScopeNode, true);
                    _curScopeNode = fakeTopicNode;
                } else {
                    return false;
                }
            });
            // must set topic node leaf property, otherwise unable to determine whether the fake topic node is the end of path.
            if (_curScopeNode.get("type") === ScopeNodeType.Topic) {
                _curScopeNode._props = {
                    leaf: _opts.isFullTopicPath
                };
            }
        }

        // modality selected
        $("a", _$modalities).click(function (e) {
            if (!_opts.enable) return false;
            e.preventDefault();
            var $item = $(this).parent("li");
            if (_$curModality[0] !== $item[0]) {
                _$modalities.not($item).removeClass("selected");
                $item.addClass("selected");
                _curModalityData = _getModalityData($item);
                _$this.trigger("modalityselect", [_curModalityData, e]);
                _$curModality = $item;
                // update products href
                _updateProductLinks();
                // update products head title
                var title = $item.data("products-head-title");
                if (title) {
                    $(".products .head-title", _$container).html(title);
                }
            }
        });

        // product tile selected
        $("> a", _$productTiles).click(function (e) {
            if (!_opts.enable) return false;
            var $item = $(this).parent("li");
            var $menu = $item.find("> ul");

            if ($menu.length > 0) {
                if ($menu.is(":visible")) {
                    $menu.hide();
                } else {
                    _$productMenus.not($menu).hide();
                    $menu.show();
                    // BUGFIX: Click event will propagate to document, and trigger click handler on document element. 
                    // So we had to use a timer to register click handler later instead of registering it here. 
                    window.setTimeout(function () {
                        $(document).one("click", function () {
                            $menu.hide();
                        });
                    }, 100);
                }
                e.preventDefault();
            } else {
                _$productTiles.not($item).removeClass("selected");
                _$productMenus.not($menu).hide();
                $item.addClass("selected");
                _changeProduct(e, $item);
            }
        });

        // register dropdown menu hide handler, if menu is already shown.
        _$productMenus.each(function () {
            var $menu = $(this);
            if ($menu.is(":visible")) {
                $(document).one("click", function () {
                    $menu.hide();
                });
            }
        });

        // sub-product selected
        $("li > a", _$productTiles).click(function (e) {
            if (!_opts.enable) return false;
            var $item = $(this).parent("li");
            var $menu = $item.parent("ul");
            var $tileItem = $menu.parent("li");
            $menu.hide();

            _$productTiles.not($tileItem).removeClass("selected");
            _$productMenus.not($menu).hide();
            $tileItem.addClass("selected");

            _changeProduct(e, $item);
        });

        _$container.on("keypress", "a", function (e) {
            if (e.which === 13) {
                $(this).click();
            }
        });
        // handle get-instances event
        $(document).on("get-instances", function (e, instances, matchFunc) {
            matchFunc = $.isFunction(matchFunc) ? matchFunc : $.isFunction(instances) ? instances : null;
            instances = $.isArray(instances) ? instances : [];
            if (matchFunc === null || matchFunc(_this) === true) {
                instances.push(_this);
            }
        });
    };

    _init();

    _this.enable = function (enable) {
        _opts.enable = enable !== false;
    };

    _this.getCt = function () {
        return _$container;
    };

    _this.getCurrentModality = function () {
        if (_curModalityData) {
            return $.extend({}, _curModalityData);
        }
        return null;
    };

    _this.getCurrentScope = function () {
        return _curScopeNode;
    };

    _this.getCurrentProduct = function () {
        return _curProductNode;
    };

    _this.changeScope = function (scope) {
        if (_curProductNode && scope instanceof ScopeNode) {
            var productNode = scope.findParentByType(ScopeNodeType.Product, true);
            var newScope = null;

            // change scope to product
            if (productNode === scope && productNode.getPath() === _curProductNode.getPath()) {
                newScope = _curProductNode;
            } else {
                // currently, we don't allow outside to change wayfinder product scope here!!
                if (productNode !== _curProductNode) {
                    if (productNode.getPath() === _curProductNode.getPath()) {
                        // when topic panel is loaded without passing current product node, 
                        // the product node will point to different address.
                        // currently, topic panel will make a fake product node with key only.
                        // we should move all current product topics to current product node, let them connect each other.
                        // P.S. if you still have any confusion about following code, please feel free to contact with v-jishao.
                        _curProductNode.removeChildren(_curProductNode.findChildrenByType(ScopeNodeType.Topic));
                        _curProductNode.addChildren(productNode.get("children"));
                    } else {
                        return false; // scope product is not match!!
                    }
                }
                newScope = scope;
            }

            if (_curScopeNode !== newScope) {
                var oldScopeNode = _curScopeNode;
                _curScopeNode = newScope;
                _updateModalityLinks();
                _$this.trigger("scopechange", [_curScopeNode, oldScopeNode]);
            }
            return true;
        }
        return false;
    };
};

Microsoft.Support.GSS.ScopeNode = function (type, key, name, props, children) {
    var _this = this;
    $.extend(_this, {
        _type: parseInt(type, 10),
        _key: key ? $.trim(key).toLowerCase() : '',
        _name: $.trim(name),
        _props: $.isPlainObject(props) ? props : {}
    });
    _this.addChildren(children);
};

$.extend(Microsoft.Support.GSS.ScopeNode.prototype, {
    _type: null,
    _key: null,
    _name: '',
    _props: {},
    _parent: null,
    _children: null,
    _filterScopeNodes: function (obj) {
        var _thisCls = this.constructor;
        if (obj) {
            if (obj instanceof _thisCls) {
                return obj;
            }
            if ($.isArray(obj)) {
                var array = [];
                $.each(obj, function (idx, val) {
                    if (val instanceof _thisCls) {
                        array.push(val);
                    }
                });
                return array;
            }
        }
        return null;
    },
    parsePath: function (path) {
        var parts = [];
        if (typeof path === 'string' && path) {
            $.each(path.split(/[\\\/]/), function (idx, part) {
                part = $.trim(part);
                if (part) {
                    parts.push(part.toLowerCase());
                }
            });
        }
        return parts;
    },
    addChildren: function (children) {
        var _this = this;
        children = _this._filterScopeNodes(children);
        if (children) {
            if (!$.isArray(_this._children)) {
                _this._children = [];    // init children array
            }

            $.each($.isArray(children) ? children : [children], function (idx, child) {
                if ($.inArray(child, _this._children) === -1) {

                    // remove child from ex-parent
                    var exParent = child.get("parent");
                    if (exParent && exParent !== _this) {
                        exParent.removeChildren(child);
                    }

                    _this._children.push(child);
                    child.setParent(_this, true);
                }
            });
        }
    },
    removeChildren: function (children) {
        var _this = this;
        $.each($.isArray(children) ? children : [children], function (i, child) {
            var idx = $.inArray(child, _this._children);
            if (idx > -1) {
                _this._children.splice(idx, 1);
                child.setParent(null);
            }
        });
    },
    clearChildren: function () {
        this._children = [];
    },
    findChildren: function (func, onlyFirst) {
        var _this = this;
        onlyFirst = onlyFirst === true;
        var found = [];
        if ($.isFunction(func) && _this._children) {
            $.each(_this._children, function (idx, child) {
                if (func(child)) {
                    found.push(child);
                    if (onlyFirst) {
                        return false;
                    }
                }
            });
        }
        return found;
    },
    findChildrenByType: function (type, onlyFirst) {
        return this.findChildren(function (child) {
            return child.get("type") === type;
        }, onlyFirst);
    },
    findChildByKey: function (key) {
        var arr = this.findChildren(function (child) {
            return child.get("key") === key;
        }, true);
        return arr.length > 0 ? arr[0] : null;
    },
    findChildByPath: function (pathOrParts, exactMatch, includSelf) {
        var _this = this;
        var parts = $.isArray(pathOrParts) ? pathOrParts : _this.parsePath(pathOrParts);
        if (includSelf === true) {
            if (parts[0] !== _this.key) {
                return null;
            }
            parts = parts.slice(1);
        }
        var node = _this;
        var i = 0;
        $.each(parts, function (idx, key) {
            var found = node.findChildByKey(key);
            if (found) {
                node = found;
            } else {
                if (exactMatch === true) {
                    node = null;
                }
                return false;
            }
        });
        return node;
    },
    findParents: function (func, includeSelf, onlyFirst) {
        var _this = this;
        onlyFirst = onlyFirst === true;
        var found = [];
        if ($.isFunction(func)) {
            var node = includeSelf === true ? _this : _this._parent;
            while (node) {
                if (func(node)) {
                    found.push(node);
                    if (onlyFirst) {
                        break;
                    }
                }
                node = node._parent;
            }
        }
        return found;
    },
    findParentByType: function (type, includeSelf) {
        var arr = this.findParents(function (parent) {
            return parent.get("type") === type;
        }, includeSelf, true);
        return arr.length > 0 ? arr[0] : 0;
    },
    get: function (key) {
        var _this = this;
        var val = _this["_" + key];
        if (val === undefined) {
            val = _this["_" + $.trim(key).toLowerCase()];
            if (val === undefined) {
                return null;
            }
        }

        if ($.isPlainObject(val)) {
            return $.extend({}, val);
        }
        if ($.isArray(val)) {
            return val.concat();
        }
        return val;
    },
    setParent: function (parent, preventAddChild) {
        var _this = this;
        parent = _this._filterScopeNodes(parent);

        // parent is not changed
        if (_this._parent === parent) {
            return;
        }

        // remove from ex-parent
        if (_this._parent) {
            _this._parent.removeChildren(_this);
            _this._parent = null;
        }

        // add into new parent
        if (parent) {
            _this._parent = parent;
            if (preventAddChild !== true) {
                parent.addChildren(_this);
            }
        }
    },
    getPathNodes: function (excludeRoot) {
        var _this = this;
        if (excludeRoot === true && _this._type === Microsoft.Support.GSS.ScopeNodeType.Root) {
            return [];
        }
        var nodes = _this._parent ? _this._parent.getPathNodes(excludeRoot) : [];
        nodes.push(_this);
        return nodes;
    },
    getPath: function () {
        var _this = this;
        var path = _this._parent ? _this._parent.getPath() : "";
        return (path ? path + "/" : "") + _this._key;
    }
});

Microsoft.Support.GSS.ScopeNodeType = {
    Root: 0,
    Product: 1,
    Topic: 2
};