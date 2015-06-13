import urllib2
import os
import json

WUP_SIM_LINK = "http://strazdas.vdu.lt:8081/AcatWSOntology4/rest/similarity"
PRAC_HOME = os.environ['PRAC_HOME']

NLTK_POS = ['n', 'v', 'a', 'r']

colorsims = {}
shapesims = {}
sizesims = {}
consistencysims = {}
dimensionsims = {}

known_concepts = ['hydrochloric_acid.n.01',
                  'drop.n.02',
                  'sodium_hydroxide.n.01',
                  'water.n.06',
                  'liter.n.01',
                  'milliliter.n.01',
                  'morsel.n.01',
                  'test_tube.n.01',
                  'bottle.n.01',
                  'petri_dish.n.01',
                  'silver_nitrate.n.01'
                  #'soup.n.01', 
#                   'milk.n.01', 
#                   'water.n.06', 
#                   'cup.n.01', 
#                   'cup.n.02', 
#                   'glass.n.02', 
#                   'bowl.n.03', 
# #                   'coffee.n.01', 
#                   'bowl.n.04',
#                   'spoon.n.01',
#                   'spoon.n.02',
#                   'sauce.n.01',
#                   'salt.n.02',
#                   'pepper.n.03',
#                   'marjoram.n.01',
#                   'rosemary.n.01',
#                   'tomato_sauce.n.01',
#                   'carbonara.n.01',
#                 'batter.n.02',
#                   'baking_tray.n.01',
#                   'cheese.n.01',
#                   'mozzarella.n.01',
#                   'water_faucet.n.01',
#                 'oven.n.01',
#                   'degree_celsius.n.01',
#                   'refrigerator.n.01',
#                   'stove.n.01'
#                 'one.n.01',
#                 'two.n.01',
#                 'three.n.01',
#                 'four.n.01',
#                 'five.n.01',
#                 'six.n.01',
#                 'seven.n.01',
#                 'eight.n.01',
#                 'nine.n.01',
                  ]


class WordNet(object):
  
    def wup_similarity(self, synset1, synset2):
        request_answer = urllib2.urlopen(WUP_SIM_LINK+"/"+synset1+"/"+synset2).read()
        sim = 0.0
        
        try:
            json_obj = json.loads(request_answer)
            sim = float(json_obj['SimilarityValue'])
        
        except Exception as e:
            #TODO add logger
            print request_answer
        
        return sim
        
        
    
            